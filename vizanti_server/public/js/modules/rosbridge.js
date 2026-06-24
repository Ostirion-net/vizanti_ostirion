import { zenoh, MessageReader, MessageWriter } from '../lib/zenoh_foxglove_bundle.js';
import { ROS2_SCHEMAS } from './schemas.js';

window.ROSLIB = {
    Ros: class {}, Topic: class { constructor(){} subscribe(){} unsubscribe(){} publish(){} },
    Message: class { constructor(data) { Object.assign(this, data); } },
    Service: class { constructor(){} callService(){} }, ServiceRequest: class { constructor(data) { Object.assign(this, data); } }
};

const paramsModule = await import(`${base_url}/ros_launch_params`);
const params = paramsModule.default;

class ZenohBridge {
    constructor(url) {
        this.url = url;
        this.port = 8000;
        this.session = null;
        this.connected = false;
        this.status = "Connecting...";
        this.domain_prefix = "0/rt"; 
        this.connect();
    }

    async connect() {
        const locator_url = `ws://${this.url}:${this.port}`;
        console.log(`Attempting Zenoh connection via locator: ${locator_url}`);
        
        try {
            // THE FIX: The Wasm API explicitly demands a single 'locator' string.
            this.session = await zenoh.open({ locator: locator_url });
            
            console.log('✅ Connected to Zenoh router natively!');
            this.connected = true;
            this.status = "Connected.";
            window.dispatchEvent(new Event('rosbridge_change'));
            
        } catch (error) {
            console.error("Zenoh connection failed (Is the router running?):", error.message || error);
            this.connected = false;
            this.status = "Connection lost.";
            window.dispatchEvent(new Event('rosbridge_change'));
            setTimeout(() => this.connect(), 2000);
        }
    }

    async subscribe(topic_name, rootMessageName, callback) {
        if (!this.session) return null;
        if (!topic_name.startsWith('/')) topic_name = '/' + topic_name;
        const zenoh_key = `${this.domain_prefix}${topic_name}`;
        const reader = new MessageReader(ROS2_SCHEMAS);

        return await this.session.declareSubscriber(zenoh_key, (sample) => {
            try { 
                const rawBytes = new Uint8Array(sample.payload);
                callback(reader.readMessage(rawBytes.subarray(4), rootMessageName)); 
            } catch (err) {}
        });
    }

    async publish(topic_name, rootMessageName, jsObject) {
        if (!this.session) return;
        if (!topic_name.startsWith('/')) topic_name = '/' + topic_name;
        const zenoh_key = `${this.domain_prefix}${topic_name}`;
        const writer = new MessageWriter(ROS2_SCHEMAS);
        
        const messageBytes = writer.writeMessage(jsObject, rootMessageName);
        const cdrHeader = new Uint8Array([0x00, 0x01, 0x00, 0x00]);
        const payload = new Uint8Array(cdrHeader.length + messageBytes.length);
        payload.set(cdrHeader); payload.set(messageBytes, cdrHeader.length);
        
        const publisher = await this.session.declarePublisher(zenoh_key);
        await publisher.put(payload);
    }
    
    async get_all_nodes() { return { nodes: [] }; } async get_all_topics() { return { topics: [], types: [] }; }
    async get_topics() { return []; } async get_services() { return []; } async get_topic_publishers_and_subscribers() { return { publishers: [], subscribers: [] }; }
}

export var rosbridge = new ZenohBridge(window.location.hostname);
