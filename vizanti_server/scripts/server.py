#!/usr/bin/env python3

import os
import base64
import hashlib
import socket
import struct
import threading
import logging
import json
import rclpy

import json
from flask import Flask, render_template, send_from_directory, make_response, request, jsonify
from waitress.server import create_server

from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pathlib import Path

node = None
param_base_url = ""
param_port = 5000
param_port_rosbridge = 5001
param_compression = "none"
param_default_widget_config = ""

def get_public_dir():
	p = Path(__file__).resolve()
	path = p.parents[1] / 'public'
	if path.exists():
		return path #for --symlink-install
	return get_package_share_directory('vizanti_server')+ '/public/'

app = Flask(__name__, static_folder=get_public_dir(), template_folder=get_public_dir())

def get_file(path):
	with open(param_default_widget_config, 'r') as f:
		file_content = f.read()
		js_module = f"const content = {json.dumps(file_content)};\nexport default content;"
		response = make_response(js_module)
		response.headers['Content-Type'] = 'application/javascript'
		return response

def get_files(path, valid_extensions):
	templates_dir = os.path.join(app.static_folder, path)
	file_list = []

	for root, dirs, files in os.walk(templates_dir):
		for file in files:
			if os.path.splitext(file)[1] in valid_extensions:
				file_path = os.path.join(root, file)
				with open(file_path, 'r') as f:
					file_content = f.read()
				file_list.append({'path': os.path.relpath(file_path, templates_dir), 'content': file_content})

	js_module = f"const files = {json.dumps(file_list)};\nexport default files;"

	#fetch workaround hackery for webkit support on HTTP
	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response

def get_paths(path, valid_extensions):
	templates_dir = os.path.join(app.static_folder, path)
	file_list = []

	for root, dirs, files in os.walk(templates_dir):
		for file in files:
			if os.path.splitext(file)[1] in valid_extensions:
				file_list.append(os.path.relpath(os.path.join(root, file), templates_dir))

	js_module = f"const paths = {json.dumps(file_list)};\nexport default paths;"

	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response

def index():
	return render_template('index.html', base_url=param_base_url)

def list_template_files():
	return get_files("templates", ['.html', '.js', '.css'])

def list_robot_model_files():
	templates_dir = os.path.join(app.static_folder, "assets/robot_model")
	categorized_files = {
		'ground': [],
		'air': [],
		'sea': [],
		'misc': []
	}
	
	for root, dirs, files in os.walk(templates_dir):
		for file in files:
			if os.path.splitext(file)[1] == '.png':
				rel_path = os.path.relpath(root, templates_dir)
				category = rel_path if rel_path in categorized_files else 'misc'
				if category == '.':  # files in root directory
					category = 'misc'
				categorized_files[category].append(file)
	
	js_module = f"const categorizedPaths = {json.dumps(categorized_files)};\nexport default categorizedPaths;"
	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response


def get_default_widget_config():
	return get_file(param_default_widget_config)

def save_config():
	if request.content_length is not None and request.content_length > 100 * 1024:
		return jsonify({"status": "error", "message": "Payload too large"}), 413
		
	try:
		config_data = request.json 
		if not config_data:
			return jsonify({"status": "error", "message": "Invalid JSON"}), 400
			
		with open(param_default_widget_config, 'w') as f:
			json.dump(config_data, f, indent=2)
		return jsonify({"status": "success"})
	except Exception as e:
		return jsonify({"status": "error", "message": str(e)}), 500

def load_config():
	try:
		with open(param_default_widget_config, 'r') as f:
			return jsonify(json.load(f))
	except Exception as e:
		return jsonify({"status": "error", "message": str(e)}), 500

def list_ros_launch_params():
	params = {
		"port": param_port,
		"port_rosbridge": param_port_rosbridge,
		"compression": param_compression,
		"namespace": os.environ.get("ROS_NAMESPACE", "")
	}
	js_module = f"const params = {json.dumps(params)};\nexport default params;"
	response = make_response(js_module)
	response.headers['Content-Type'] = 'application/javascript'
	return response

def serve_static(path):
	return send_from_directory(app.static_folder, path)

class ServerThread(threading.Thread):
	def __init__(self, app, host='0.0.0.0', port=5000):
		threading.Thread.__init__(self)
		self.daemon = True

		self.log = logging.getLogger('waitress')
		self.log.setLevel(logging.INFO)
		handler = logging.StreamHandler()
		handler.setFormatter(logging.Formatter(
			'[%(levelname)s] [%(asctime)s] [waitress]: %(message)s '
		))
		self.log.addHandler(handler)

		self.app = app
		self.host = host
		self.port = port
		self.ctx = app.app_context()
		self.ctx.push()
		
		self._server = None
		self._stop_event = threading.Event()

	def run(self):
		self._server = create_server(
				self.app,
				host=self.host,
				port=self.port,
				threads=16,
				connection_limit=200,
				channel_timeout=30
			)
		try:
			self._server.run()
		except KeyboardInterrupt:
			self.shutdown()
		
	def shutdown(self):
		if self._server:
			self._server.close()  # This triggers waitress to stop accepting new connections
			self._stop_event.set()  # Signal that we're stopping
			rospy.loginfo("Waitress server shutting down...")






class VizantiSocketThread(threading.Thread):
	OP_GET_TOPICS = 1
	OP_TOPIC_LIST = 2
	OP_SUBSCRIBE = 3
	OP_MESSAGE = 4
	OP_SUBSCRIBE_ACK = 5
	OP_PUBLISH = 6

	def __init__(self, host, port, node, qos_depth):
		threading.Thread.__init__(self)
		self.daemon = True
		self.host = host
		self.port = port
		self.node = node
		self.qos_depth = qos_depth
		self.stop_event = threading.Event()
		self.sock = None
		self.lock = threading.Lock()
		self.client_topics = {}
		self.subscriptions = {}
		self.publishers = {}

	def run(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.sock.bind((self.host, self.port))
		self.sock.listen()
		self.node.get_logger().info(
			f"Vizanti socket running at ws://{self.host}:{self.port}"
		)
		while not self.stop_event.is_set():
			try:
				client, _ = self.sock.accept()
				with self.lock:
					self.client_topics[client] = set()
				threading.Thread(
					target=self.handle_client,
					args=(client,),
					daemon=True
				).start()
			except OSError:
				break

	def handle_client(self, client):
		try:
			request = client.recv(4096).decode("utf-8", "ignore")
			key = self.get_header(request, "Sec-WebSocket-Key")
			if key == "":
				client.close()
				return
			client.sendall(self.handshake(self.make_accept(key)))
			while not self.stop_event.is_set():
				payload = self.read_ws_frame(client)
				if payload is None:
					break
				self.handle_payload(client, payload)
		except OSError:
			pass
		finally:
			with self.lock:
				self.client_topics.pop(client, None)
			client.close()

	def handle_payload(self, client, payload):
		if len(payload) < 9:
			return
		op, request_id, size = struct.unpack(">BII", payload[:9])
		body = payload[9:]
		if len(body) != size:
			return
		if op == self.OP_GET_TOPICS:
			self.send_topic_list(client, request_id)
		if op == self.OP_SUBSCRIBE:
			topic, type_name = self.parse_topic_type(body)
			self.add_client_topic(client, topic)
			self.ensure_subscription(topic, type_name)
			header = struct.pack(">BII", self.OP_SUBSCRIBE_ACK, request_id, 0)
			self.send_ws_frame(client, header)
		if op == self.OP_PUBLISH:
			topic, type_name, raw = self.parse_publish_payload(body)
			self.publish_raw(topic, type_name, raw)

	def parse_topic_type(self, body):
		offset = 0
		topic_len = struct.unpack(">H", body[offset:offset + 2])[0]
		offset += 2
		topic = body[offset:offset + topic_len].decode("utf-8")
		offset += topic_len
		type_len = struct.unpack(">H", body[offset:offset + 2])[0]
		offset += 2
		type_name = body[offset:offset + type_len].decode("utf-8")
		return topic, type_name

	def parse_publish_payload(self, body):
		offset = 0
		topic_len = struct.unpack(">H", body[offset:offset + 2])[0]
		offset += 2
		topic = body[offset:offset + topic_len].decode("utf-8")
		offset += topic_len
		type_len = struct.unpack(">H", body[offset:offset + 2])[0]
		offset += 2
		type_name = body[offset:offset + type_len].decode("utf-8")
		offset += type_len
		raw_len = struct.unpack(">I", body[offset:offset + 4])[0]
		offset += 4
		raw = body[offset:offset + raw_len]
		if len(raw) != raw_len:
			raise ValueError("publish payload size mismatch")
		return topic, type_name, raw

	def ensure_publisher(self, topic, type_name):
		key = (topic, type_name)
		with self.lock:
			if key in self.publishers:
				return self.publishers[key]
		msg_type = get_message(type_name)
		pub = self.node.create_publisher(
			msg_type,
			topic,
			self.qos_depth
		)
		with self.lock:
			self.publishers[key] = pub
		self.node.get_logger().info(
			f"Vizanti publisher active: {topic}"
		)
		return pub

	def publish_raw(self, topic, type_name, raw):
		msg_type = get_message(type_name)
		msg = deserialize_message(raw, msg_type)
		pub = self.ensure_publisher(topic, type_name)
		pub.publish(msg)

	def add_client_topic(self, client, topic):
		with self.lock:
			if client in self.client_topics:
				self.client_topics[client].add(topic)

	def ensure_subscription(self, topic, type_name):
		with self.lock:
			if topic in self.subscriptions:
				return
		msg_type = get_message(type_name)

		def callback(raw):
			self.forward_raw(topic, type_name, bytes(raw))

		sub = self.node.create_subscription(
			msg_type,
			topic,
			callback,
			self.qos_depth,
			raw=True
		)
		with self.lock:
			self.subscriptions[topic] = sub
		self.node.get_logger().info(
			f"Vizanti raw subscription active: {topic}"
		)

	def forward_raw(self, topic, type_name, raw):
		topic_bytes = topic.encode("utf-8")
		type_bytes = type_name.encode("utf-8")
		body = b"".join([
			struct.pack(">H", len(topic_bytes)),
			topic_bytes,
			struct.pack(">H", len(type_bytes)),
			type_bytes,
			struct.pack(">I", len(raw)),
			raw
		])
		header = struct.pack(">BII", self.OP_MESSAGE, 0, len(body))
		frame = header + body
		dead = []
		with self.lock:
			clients = list(self.client_topics.items())
		for client, topics in clients:
			if topic not in topics:
				continue
			try:
				self.send_ws_frame(client, frame)
			except OSError:
				dead.append(client)
		if not dead:
			return
		with self.lock:
			for client in dead:
				self.client_topics.pop(client, None)

	def send_topic_list(self, client, request_id):
		parts = []
		pairs = []
		for name, type_list in self.node.get_topic_names_and_types():
			for type_name in type_list:
				pairs.append((name, type_name))
		if len(pairs) > 65535:
			raise ValueError("too many topics")
		parts.append(struct.pack(">H", len(pairs)))
		for name, type_name in pairs:
			name_bytes = name.encode("utf-8")
			type_bytes = type_name.encode("utf-8")
			if len(name_bytes) > 65535:
				raise ValueError("topic name too long")
			if len(type_bytes) > 65535:
				raise ValueError("topic type too long")
			parts.append(struct.pack(">H", len(name_bytes)))
			parts.append(name_bytes)
			parts.append(struct.pack(">H", len(type_bytes)))
			parts.append(type_bytes)
		body = b"".join(parts)
		header = struct.pack(
			">BII",
			self.OP_TOPIC_LIST,
			request_id,
			len(body)
		)
		self.send_ws_frame(client, header + body)

	def read_ws_frame(self, client):
		header = client.recv(2)
		if len(header) < 2:
			return None
		opcode = header[0] & 15
		length = header[1] & 127
		if length == 126:
			length = int.from_bytes(client.recv(2), "big")
		elif length == 127:
			length = int.from_bytes(client.recv(8), "big")
		mask = b""
		if header[1] & 128:
			mask = client.recv(4)
		payload = b""
		while len(payload) < length:
			chunk = client.recv(length - len(payload))
			if chunk == b"":
				return None
			payload += chunk
		if mask:
			payload = bytes(
				value ^ mask[index % 4]
				for index, value in enumerate(payload)
			)
		if opcode == 8:
			return None
		if opcode != 2:
			return b""
		return payload

	def send_ws_frame(self, client, payload):
		header = bytearray()
		header.append(130)
		size = len(payload)
		if size <= 125:
			header.append(size)
		elif size <= 65535:
			header.append(126)
			header.extend(size.to_bytes(2, "big"))
		else:
			header.append(127)
			header.extend(size.to_bytes(8, "big"))
		client.sendall(bytes(header) + payload)

	def shutdown(self):
		self.stop_event.set()
		if self.sock:
			self.sock.close()

	def get_header(self, request, name):
		for line in request.split("\r\n"):
			if line.lower().startswith(name.lower() + ":"):
				return line.split(":", 1)[1].strip()
		return ""

	def make_accept(self, key):
		guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
		raw = hashlib.sha1((key + guid).encode("ascii")).digest()
		return base64.b64encode(raw).decode("ascii")

	def handshake(self, accept):
		lines = [
			"HTTP/1.1 101 Switching Protocols",
			"Upgrade: websocket",
			"Connection: Upgrade",
			f"Sec-WebSocket-Accept: {accept}",
			"",
			""
		]
		return "\r\n".join(lines).encode("ascii")


def main(args=None):
	global node, param_base_url, param_port, param_port_rosbridge, param_compression, param_default_widget_config

	rclpy.init(args=args)
	node = rclpy.create_node('vizanti_flask_node')

	node.declare_parameter('host', '0.0.0.0')
	node.declare_parameter('port', param_port)
	node.declare_parameter('port_rosbridge', param_port_rosbridge)
	node.declare_parameter('flask_debug', True)
	node.declare_parameter('base_url', param_base_url)
	node.declare_parameter('compression', param_compression)
	node.declare_parameter('default_widget_config',param_default_widget_config)
	node.declare_parameter('vizanti_socket_qos_depth')

	param_host = node.get_parameter('host').value
	param_port = node.get_parameter('port').value
	param_port_rosbridge = node.get_parameter('port_rosbridge').value
	param_base_url = node.get_parameter('base_url').value
	param_compression = node.get_parameter('compression').value
	param_default_widget_config = node.get_parameter('default_widget_config').value
	param_socket_qos_depth = int(node.get_parameter('vizanti_socket_qos_depth').value)
	if param_socket_qos_depth <= 0:
		raise RuntimeError('Invalid vizanti_socket_qos_depth')

	if param_default_widget_config != "":
		param_default_widget_config = os.path.expanduser(param_default_widget_config)
	else:
		param_default_widget_config = os.path.join(app.static_folder, "assets/default_layout.json")

	node.get_logger().info(f"Default widget config set to {param_default_widget_config}")

	app.debug = node.get_parameter('flask_debug').value
	app.add_url_rule(param_base_url + '/', 'index', index)
	app.add_url_rule(param_base_url + '/templates/files', 'list_template_files', list_template_files)
	app.add_url_rule(param_base_url + '/assets/robot_model/paths', 'list_robot_model_files', list_robot_model_files)
	app.add_url_rule(param_base_url + '/ros_launch_params', 'ros_launch_params', list_ros_launch_params)
	app.add_url_rule(param_base_url + '/default_widget_config', 'get_default_widget_config', get_default_widget_config)
	app.add_url_rule(param_base_url + '/save_config', 'save_config', save_config, methods=['POST'])
	app.add_url_rule(param_base_url + '/load_config', 'load_config', load_config, methods=['GET'])
	app.add_url_rule(param_base_url + '/<path:path>', 'serve_static', serve_static)

	server = ServerThread(app, param_host, param_port)
	server.start()

	vizanti_socket = VizantiSocketThread(
		param_host,
		param_port_rosbridge,
		node,
		param_socket_qos_depth
	)
	vizanti_socket.start()

	node.get_logger().info(f"Flask server running at http://{param_host}:{param_port}{param_base_url}")
	node.get_logger().info(f"Public directory set as {get_public_dir()}")

	rclpy.spin(node)

	vizanti_socket.shutdown()
	vizanti_socket.join()
	server.shutdown()
	server.join()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
