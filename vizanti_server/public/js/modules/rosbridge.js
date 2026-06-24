import { CdrReader } from "../lib/zenoh_foxglove_bundle.js";
const paramsModule = await import(`${base_url}/ros_launch_params`);
const params = paramsModule.default;

const OP_GET_TOPICS = 1;
const OP_TOPIC_LIST = 2;
const OP_SUBSCRIBE = 3;
const OP_MESSAGE = 4;
const OP_SUBSCRIBE_ACK = 5;
const OP_PUBLISH = 6;

if (!window.ROSLIB) {
    window.ROSLIB = {};
}

window.ROSLIB.Topic = class {
    constructor(args) {
        this.args = args;
        this.subscription = null;
        this.subscription_promise = null;
        this.unsubscribed = false;
    }

    subscribe(callback) {
        const topicName = this.args.name;
        const typeName = this.args.messageType;

        if (!topicName) {
            throw new Error("ROSLIB.Topic missing name");
        }
        if (!typeName) {
            throw new Error("ROSLIB.Topic missing messageType");
        }

        this.unsubscribed = false;
        this.subscription_promise = rosbridge
            .subscribe(topicName, typeName, callback)
            .then(subscription => {
                if (this.unsubscribed) {
                    subscription.unsubscribe();
                    return null;
                }
                this.subscription = subscription;
                return subscription;
            })
            .catch(error => {
                console.error(error);
                throw error;
            });
    }

    unsubscribe() {
        this.unsubscribed = true;
        if (this.subscription) {
            this.subscription.unsubscribe();
            this.subscription = null;
        }
    }

    publish(message) {
        return rosbridge.publish(
            this.args.name,
            this.args.messageType,
            message
        );
    }
};

window.ROSLIB.Message = class {
    constructor(data) {
        Object.assign(this, data);
    }
};

class VizantiBridge {
    constructor(url) {
        this.url = url;
        if (!Object.prototype.hasOwnProperty.call(
            params,
            "port_rosbridge"
        )) {
            throw new Error("Missing ros_launch_params.port_rosbridge");
        }
        this.socket_port = params.port_rosbridge;
        this.socket = null;
        this.next_request_id = 1;
        this.pending_requests = new Map();
        this.topic_callbacks = new Map();
        this.connected = false;
        this.status = "Connecting...";
        this.connect();
    }

    connect() {
        const url = "ws://" + this.url + ":" + this.socket_port;
        this.status = "Connecting...";
        window.dispatchEvent(new Event("rosbridge_change"));
        this.socket = new WebSocket(url);
        this.socket.binaryType = "arraybuffer";
        this.socket.onopen = () => {
            this.connected = true;
            this.status = "Connected.";
            window.dispatchEvent(new Event("rosbridge_change"));
        };
        this.socket.onmessage = (event) => {
            if (!(event.data instanceof ArrayBuffer)) {
                return;
            }
            this.handleBinary(event.data);
        };
        this.socket.onclose = () => {
            this.connected = false;
            this.status = "Connection lost.";
            window.dispatchEvent(new Event("rosbridge_change"));
        };
        this.socket.onerror = () => {
            this.connected = false;
            this.status = "Connection error.";
            window.dispatchEvent(new Event("rosbridge_change"));
        };
    }

    async waitOpen() {
        if (this.socket.readyState === WebSocket.OPEN) {
            return;
        }
        return new Promise((resolve, reject) => {
            this.socket.addEventListener("open", resolve, { once: true });
            this.socket.addEventListener("error", reject, { once: true });
            this.socket.addEventListener("close", reject, { once: true });
        });
    }

    async request(op, payload) {
        await this.waitOpen();
        const requestId = this.next_request_id++;
        const header = new Uint8Array(9);
        const view = new DataView(header.buffer);
        view.setUint8(0, op);
        view.setUint32(1, requestId, false);
        view.setUint32(5, payload.byteLength, false);
        const frame = new Uint8Array(9 + payload.byteLength);
        frame.set(header, 0);
        frame.set(payload, 9);
        return new Promise((resolve, reject) => {
            this.pending_requests.set(requestId, { resolve, reject });
            this.socket.send(frame);
        });
    }

    handleBinary(buffer) {
        if (buffer.byteLength < 9) {
            return;
        }
        const view = new DataView(buffer);
        const op = view.getUint8(0);
        const requestId = view.getUint32(1, false);
        const size = view.getUint32(5, false);
        if (buffer.byteLength !== 9 + size) {
            return;
        }
        if (op === OP_MESSAGE) {
            this.handleMessageFrame(buffer.slice(9));
            return;
        }
        const pending = this.pending_requests.get(requestId);
        if (!pending) {
            return;
        }
        this.pending_requests.delete(requestId);
        if (!pending) {
            return;
        }
        this.pending_requests.delete(requestId);
        if (op === OP_TOPIC_LIST) {
            pending.resolve(this.decodeTopicList(buffer.slice(9)));
            return;
        }
        if (op === OP_SUBSCRIBE_ACK) {
            pending.resolve({ ok: true });
            return;
        }
        pending.reject(new Error("Unexpected binary response"));
    }

    decodeTopicList(buffer) {
        const decoder = new TextDecoder();
        const view = new DataView(buffer);
        let offset = 0;
        const count = view.getUint16(offset, false);
        offset += 2;
        const topics = [];
        const types = [];
        for (let index = 0; index < count; index++) {
            const nameLength = view.getUint16(offset, false);
            offset += 2;
            const name = decoder.decode(
                new Uint8Array(buffer, offset, nameLength)
            );
            offset += nameLength;
            const typeLength = view.getUint16(offset, false);
            offset += 2;
            const typeName = decoder.decode(
                new Uint8Array(buffer, offset, typeLength)
            );
            offset += typeLength;
            topics.push(name);
            types.push(typeName);
        }
        return { topics: topics, types: types };
    }

    encodePublishPayload(topicName, typeName, raw) {
        const encoder = new TextEncoder();
        const topicBytes = encoder.encode(topicName);
        const typeBytes = encoder.encode(typeName);
        const body = new Uint8Array(
            2 + topicBytes.length +
            2 + typeBytes.length +
            4 + raw.length
        );
        const view = new DataView(body.buffer);
        let offset = 0;

        view.setUint16(offset, topicBytes.length, false);
        offset += 2;
        body.set(topicBytes, offset);
        offset += topicBytes.length;

        view.setUint16(offset, typeBytes.length, false);
        offset += 2;
        body.set(typeBytes, offset);
        offset += typeBytes.length;

        view.setUint32(offset, raw.length, false);
        offset += 4;
        body.set(raw, offset);

        return body;
    }

    encodeTwist(message) {
        if (!message || !message.linear || !message.angular) {
            throw new Error("Invalid Twist message");
        }

        const raw = new Uint8Array(52);
        const view = new DataView(raw.buffer);

        raw[0] = 0;
        raw[1] = 1;
        raw[2] = 0;
        raw[3] = 0;

        let offset = 4;
        view.setFloat64(offset, message.linear.x, true);
        offset += 8;
        view.setFloat64(offset, message.linear.y, true);
        offset += 8;
        view.setFloat64(offset, message.linear.z, true);
        offset += 8;
        view.setFloat64(offset, message.angular.x, true);
        offset += 8;
        view.setFloat64(offset, message.angular.y, true);
        offset += 8;
        view.setFloat64(offset, message.angular.z, true);

        return raw;
    }

    cdrAlign(offset, alignment) {
        const remainder = offset % alignment;
        if (remainder === 0) {
            return offset;
        }
        return offset + alignment - remainder;
    }

    requireFiniteNumber(value, name) {
        if (typeof value !== "number" || !Number.isFinite(value)) {
            throw new Error("Invalid numeric field: " + name);
        }
        return value;
    }

    encodedStringSize(value) {
        const bytes = new TextEncoder().encode(value);
        return 4 + this.cdrAlign(bytes.length + 1, 4);
    }

    writeString(raw, view, offset, value) {
        const bytes = new TextEncoder().encode(value);
        view.setUint32(4 + offset, bytes.length + 1, true);
        offset += 4;
        raw.set(bytes, 4 + offset);
        offset += bytes.length;
        raw[4 + offset] = 0;
        offset += 1;
        return this.cdrAlign(offset, 4);
    }

    writePoseAt(raw, view, offset, pose) {
        if (!pose || !pose.position || !pose.orientation) {
            throw new Error("Invalid Pose message");
        }

        offset = this.cdrAlign(offset, 8);

        view.setFloat64(4 + offset, this.requireFiniteNumber(pose.position.x, "pose.position.x"), true);
        offset += 8;
        view.setFloat64(4 + offset, this.requireFiniteNumber(pose.position.y, "pose.position.y"), true);
        offset += 8;
        view.setFloat64(4 + offset, this.requireFiniteNumber(pose.position.z, "pose.position.z"), true);
        offset += 8;

        view.setFloat64(4 + offset, this.requireFiniteNumber(pose.orientation.x, "pose.orientation.x"), true);
        offset += 8;
        view.setFloat64(4 + offset, this.requireFiniteNumber(pose.orientation.y, "pose.orientation.y"), true);
        offset += 8;
        view.setFloat64(4 + offset, this.requireFiniteNumber(pose.orientation.z, "pose.orientation.z"), true);
        offset += 8;
        view.setFloat64(4 + offset, this.requireFiniteNumber(pose.orientation.w, "pose.orientation.w"), true);
        offset += 8;

        return offset;
    }

    encodePoseStamped(message) {
        if (!message || !message.pose) {
            throw new Error("Invalid PoseStamped message");
        }

        const header = message.header || {};
        const stamp = header.stamp || {};
        const sec = stamp.sec ?? stamp.secs ?? 0;
        const nanosec = stamp.nanosec ?? stamp.nsecs ?? 0;
        const frameId = header.frame_id ?? "map";

        let offset = 0;
        offset += 4;
        offset += 4;
        offset = this.cdrAlign(offset, 4);
        offset += this.encodedStringSize(frameId);
        offset = this.cdrAlign(offset, 8);
        offset += 56;

        const raw = new Uint8Array(4 + offset);
        const view = new DataView(raw.buffer);

        raw[0] = 0;
        raw[1] = 1;
        raw[2] = 0;
        raw[3] = 0;

        offset = 0;
        view.setInt32(4 + offset, sec, true);
        offset += 4;
        view.setUint32(4 + offset, nanosec, true);
        offset += 4;
        offset = this.writeString(raw, view, offset, frameId);
        this.writePoseAt(raw, view, offset, message.pose);

        return raw;
    }

    encodeMessage(typeName, message) {
        if (typeName === "geometry_msgs/msg/Twist") {
            return this.encodeTwist(message);
        }
        if (typeName === "geometry_msgs/msg/PoseStamped") {
            return this.encodePoseStamped(message);
        }
        throw new Error("Publish type not supported: " + typeName);
    }

    waitSocketOpen() {
        if (!this.socket) {
            throw new Error("WebSocket not created");
        }

        if (this.socket.readyState === WebSocket.OPEN) {
            return Promise.resolve();
        }

        if (this.socket.readyState !== WebSocket.CONNECTING) {
            throw new Error("WebSocket is not open");
        }

        return new Promise((resolve, reject) => {
            const onOpen = () => {
                cleanup();
                resolve();
            };

            const onError = () => {
                cleanup();
                reject(new Error("WebSocket open failed"));
            };

            const onClose = () => {
                cleanup();
                reject(new Error("WebSocket closed before open"));
            };

            const cleanup = () => {
                this.socket.removeEventListener("open", onOpen);
                this.socket.removeEventListener("error", onError);
                this.socket.removeEventListener("close", onClose);
            };

            this.socket.addEventListener("open", onOpen);
            this.socket.addEventListener("error", onError);
            this.socket.addEventListener("close", onClose);
        });
    }

    async publishRaw(topicName, typeName, message) {
        if (!topicName) {
            throw new Error("publish missing topic name");
        }
        if (!typeName) {
            throw new Error("publish missing message type");
        }
        await this.waitOpen();
        const raw = this.encodeMessage(typeName, message);
        const payload = this.encodePublishPayload(topicName, typeName, raw);
        const frame = new Uint8Array(9 + payload.length);
        const view = new DataView(frame.buffer);
        view.setUint8(0, OP_PUBLISH);
        view.setUint32(1, 0, false);
        view.setUint32(5, payload.length, false);
        frame.set(payload, 9);
        this.socket.send(frame);
    }

    encodeTopicType(topicName, typeName) {
        const encoder = new TextEncoder();
        const topicBytes = encoder.encode(topicName);
        const typeBytes = encoder.encode(typeName);
        const payload = new Uint8Array(
            4 + topicBytes.byteLength + typeBytes.byteLength
        );
        const view = new DataView(payload.buffer);
        let offset = 0;
        view.setUint16(offset, topicBytes.byteLength, false);
        offset += 2;
        payload.set(topicBytes, offset);
        offset += topicBytes.byteLength;
        view.setUint16(offset, typeBytes.byteLength, false);
        offset += 2;
        payload.set(typeBytes, offset);
        return payload;
    }


    decodeMessage(topicName, typeName, raw) {
        if (typeName === "tf2_msgs/msg/TFMessage") {
            return this.decodeTFMessage(raw);
        }
        if (typeName === "sensor_msgs/msg/LaserScan") {
            return this.decodeLaserScan(raw);
        }
        if (typeName === "nav_msgs/msg/OccupancyGrid") {
            return this.decodeOccupancyGrid(raw);
        }
        if (typeName === "nav_msgs/msg/Path") {
            return this.decodePath(raw);
        }
        if (typeName === "sensor_msgs/msg/CompressedImage") {
            return this.decodeCompressedImage(raw);
        }
        if (typeName === "sensor_msgs/msg/Imu") {
            return this.decodeImu(raw);
        }
        if (typeName === "sensor_msgs/msg/BatteryState") {
            return this.decodeBatteryState(raw);
        }
        return { topic: topicName, type: typeName, payload: raw };
    }

    decodeTFMessage(raw) {
        const reader = new CdrReader(new Uint8Array(raw));
        const transforms = [];
        const count = reader.sequenceLength();
        for (let i = 0; i < count; i += 1) {
            transforms.push(this.decodeTransformStamped(reader));
        }
        return { transforms: transforms };
    }

    decodeTransformStamped(reader) {
        return {
            header: this.decodeHeader(reader),
            child_frame_id: reader.string(),
            transform: this.decodeTransform(reader)
        };
    }

    decodeHeader(reader) {
        return {
            stamp: {
                sec: reader.int32(),
                nanosec: reader.uint32()
            },
            frame_id: reader.string()
        };
    }

    decodeTransform(reader) {
        return {
            translation: this.decodeVector3(reader),
            rotation: this.decodeQuaternion(reader)
        };
    }

    decodeVector3(reader) {
        return {
            x: reader.float64(),
            y: reader.float64(),
            z: reader.float64()
        };
    }

    decodeQuaternion(reader) {
        return {
            x: reader.float64(),
            y: reader.float64(),
            z: reader.float64(),
            w: reader.float64()
        };
    }

    decodeLaserScan(raw) {
        const reader = new CdrReader(new Uint8Array(raw));
        return {
            header: this.decodeHeader(reader),
            angle_min: reader.float32(),
            angle_max: reader.float32(),
            angle_increment: reader.float32(),
            time_increment: reader.float32(),
            scan_time: reader.float32(),
            range_min: reader.float32(),
            range_max: reader.float32(),
            ranges: this.decodeFloat32Sequence(reader),
            intensities: this.decodeFloat32Sequence(reader)
        };
    }

    decodeOccupancyGrid(raw) {
        const reader = new CdrReader(new Uint8Array(raw));
        return {
            header: this.decodeHeader(reader),
            info: this.decodeMapMetaData(reader),
            data: this.decodeInt8Sequence(reader)
        };
    }

    decodeMapMetaData(reader) {
        return {
            map_load_time: {
                sec: reader.int32(),
                nanosec: reader.uint32()
            },
            resolution: reader.float32(),
            width: reader.uint32(),
            height: reader.uint32(),
            origin: this.decodePose(reader)
        };
    }

    decodePath(raw) {
        const reader = new CdrReader(new Uint8Array(raw));
        const path = {
            header: this.decodeHeader(reader),
            poses: []
        };
        const count = reader.sequenceLength();
        for (let i = 0; i < count; i += 1) {
            path.poses.push(this.decodePoseStamped(reader));
        }
        return path;
    }

    decodePoseStamped(reader) {
        return {
            header: this.decodeHeader(reader),
            pose: this.decodePose(reader)
        };
    }

    decodeCompressedImage(raw) {
        const reader = new CdrReader(new Uint8Array(raw));
        return {
            header: this.decodeHeader(reader),
            format: reader.string(),
            data: this.decodeUint8Sequence(reader)
        };
    }

    decodePose(reader) {
        return {
            position: this.decodePoint(reader),
            orientation: this.decodeQuaternion(reader)
        };
    }

    decodePoint(reader) {
        return {
            x: reader.float64(),
            y: reader.float64(),
            z: reader.float64()
        };
    }

    decodeFloat32Sequence(reader) {
        const length = reader.sequenceLength();
        return reader.float32Array(length);
    }

    decodeUint8Sequence(reader) {
        const length = reader.sequenceLength();
        return reader.uint8Array(length);
    }

    decodeInt8Sequence(reader) {
        const length = reader.sequenceLength();
        return reader.int8Array(length);
    }

    decodeImu(raw) {
        const reader = new CdrReader(new Uint8Array(raw));
        return {
            header: this.decodeHeader(reader),
            orientation: this.decodeQuaternion(reader),
            orientation_covariance: reader.float64Array(9),
            angular_velocity: this.decodeVector3(reader),
            angular_velocity_covariance: reader.float64Array(9),
            linear_acceleration: this.decodeVector3(reader),
            linear_acceleration_covariance: reader.float64Array(9)
        };
    }

    decodeBatteryState(raw) {
        const reader = new CdrReader(new Uint8Array(raw));
        return {
            header: this.decodeHeader(reader),
            voltage: reader.float32(),
            temperature: reader.float32(),
            current: reader.float32(),
            charge: reader.float32(),
            capacity: reader.float32(),
            design_capacity: reader.float32(),
            percentage: reader.float32(),
            power_supply_status: reader.uint8(),
            power_supply_health: reader.uint8(),
            power_supply_technology: reader.uint8(),
            present: reader.uint8() !== 0,
            cell_voltage: this.decodeFloat32Sequence(reader),
            location: reader.string(),
            serial_number: reader.string()
        };
    }

    handleMessageFrame(buffer) {
        const decoder = new TextDecoder();
        const view = new DataView(buffer);
        let offset = 0;
        const topicLength = view.getUint16(offset, false);
        offset += 2;
        const topicName = decoder.decode(
            new Uint8Array(buffer, offset, topicLength)
        );
        offset += topicLength;
        const typeLength = view.getUint16(offset, false);
        offset += 2;
        const typeName = decoder.decode(
            new Uint8Array(buffer, offset, typeLength)
        );
        offset += typeLength;
        const rawLength = view.getUint32(offset, false);
        offset += 4;
        const raw = buffer.slice(offset, offset + rawLength);
        const callback = this.topic_callbacks.get(topicName);
        if (callback) {
            callback(this.decodeMessage(topicName, typeName, raw));
        }
    }

    normalizeType(typeName) {
        return typeName.replace("/msg/", "/");
    }

    async get_all_nodes() {
        return { nodes: [] };
    }

    async get_all_topics() {
        return await this.request(OP_GET_TOPICS, new Uint8Array());
    }

    async get_topics(typeName) {
        const all = await this.get_all_topics();
        const topics = [];
        const expected = this.normalizeType(typeName);
        for (let index = 0; index < all.topics.length; index++) {
            if (this.normalizeType(all.types[index]) === expected) {
                topics.push(all.topics[index]);
            }
        }
        return topics;
    }

    async get_services(typeName) {
        return [];
    }

    async subscribe(topicName, rootMessageName, callback) {
        const all = await this.get_all_topics();
        let typeName = "";
        for (let index = 0; index < all.topics.length; index++) {
            if (all.topics[index] === topicName) {
                typeName = all.types[index];
                break;
            }
        }
        if (typeName === "") {
            throw new Error("Topic not found: " + topicName);
        }
        this.topic_callbacks.set(topicName, callback);
        await this.request(
            OP_SUBSCRIBE,
            this.encodeTopicType(topicName, typeName)
        );
        return {
            unsubscribe: () => this.topic_callbacks.delete(topicName)
        };
    }

    async publish(topicName, rootMessageName, jsObject) {
        return this.publishRaw(topicName, rootMessageName, jsObject);
    }
}

export var rosbridge = new VizantiBridge(window.location.hostname);
