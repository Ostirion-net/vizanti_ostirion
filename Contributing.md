# Contributing to Vizanti Ostirion

Got an idea on how to improve Vizanti Ostirion?

Contributions are welcome. Feel free to fork the repository and submit pull requests for fixes, documentation improvements, ROS 2 integration work, transport improvements, or extra widgets you find useful.

## Overview

Vizanti Ostirion is split into a ROS 2 server side and a web browser client.

### Server

The static content is served using Flask, with Jinja2 as the template engine.

ROS 2 topic communication for the standard Ostirion runtime goes through the Vizanti Ostirion binary WebSocket bridge implemented in the server and browser bridge module.

[server.py](vizanti_server/scripts/server.py)

The service handler provides server-side ROS 2 utility operations such as node management, map operations, bag recording, package discovery, lifecycle discovery, and parameter access.

[service_handler.py](vizanti_server/scripts/service_handler.py)

The TF consolidator groups TF data for efficient browser-side visualization.

[tf_consolidator.cpp](vizanti_cpp/src/tf_consolidator.cpp)

The Nav2 bridge connects Vizanti waypoint interaction to Nav2 mission execution.

[vizanti_nav2_bridge.py](vizanti_server/scripts/vizanti_nav2_bridge.py)

### Client

The front end is written in vanilla JavaScript with ES6 modules, while making use of Jinja templating and fixed browser-side dependencies in [vizanti_server/public/js/lib](vizanti_server/public/js/lib).

The main browser-side ROS communication module is:

[rosbridge.js](vizanti_server/public/js/modules/rosbridge.js)

The file name is kept for compatibility with the original Vizanti frontend structure. In Vizanti Ostirion, this module provides a ROSLIB-compatible topic interface on top of the Ostirion binary WebSocket bridge.

---

## Adding a new widget

Upon client load, the server collects all widget templates in [vizanti_server/public/templates](vizanti_server/public/templates) and sends them to the client where they are processed by [main.js](vizanti_server/public/js/main.js).

As per the existing examples, the supported files are:

- `name_icon.html`  
  The mandatory icon element that will be added to the taskbar.

- `name_modal.html`  
  A popup window element used to select the topic and change widget settings.

- `name_view.html`  
  A canvas layer for drawing visualizers. Define the Z-order value so that your element is rendered in the correct order. Canvases of the same widget type are rendered in the order they were added.

- `name_script.js`  
  A JavaScript module that can import helper singletons from [vizanti_server/public/js/modules](vizanti_server/public/js/modules).

To add the widget to the taskbar, it has to be defined in the `add_types_container` list in [add_modal.html](vizanti_server/public/templates/add/add_modal.html). The `data-topic` value is used for creating widgets from a specific topic.

Any required assets should be placed in [vizanti_server/public/assets](vizanti_server/public/assets).

Any SVG icons that need to display a colour change using `utilModule.setIconColor` need to have specific elements tagged as `id="fillColor"` or `id="strokeColor"`, depending on which part needs coloring.

### {uniqueID}

Every widget should have the string `{uniqueID}` embedded in its code. It will be replaced with a unique sequential identifier at widget instantiation, for example `auto53`.

This allows scattered icon, modal, view, and script elements to reference each other directly and keep unique saved settings.

Example:

```javascript
// battery_icon.html
<div id="{uniqueID}_icon" class="icon noselect">
	<img src="assets/battery_unknown.svg" alt="?" width="50" height="50" onclick="openModal('{uniqueID}_modal')">
</div>
```

Which can then be obtained from the script as follows:

```javascript
// battery_script.js
const icon = document.getElementById("{uniqueID}_icon");
icon.src = "something.png";
```

-----

## Helper Singletons

Quaternion.js is loaded globally.

The browser communication module exposes a ROSLIB-compatible topic interface for widgets through `window.ROSLIB.Topic` and `window.ROSLIB.Message`.

There are also two global functions to streamline opening and closing modals: `openModal(widget_id+"_modal")` and `closeModal(widget_id+"_modal")`, found in [setup.js](vizanti_server/public/js/setup.js). They are typically called when an icon is clicked.

### View

```javascript
let viewModule = await import(`${base_url}/js/modules/view.js`);
let view = viewModule.view;
```

Handles how the screen is moved and zoomed by the user for final rendering from ROS TF space into screen space. The TF and screen frames are currently not rotated, which simplifies rendering.

Example:

```javascript
const img_width = view.getMapUnitsInPixels(width_meters);
const img_height = view.getMapUnitsInPixels(length_meters);

let pos = view.fixedToScreen({
    x: object.translation.x,
    y: object.translation.y,
});

ctx.translate(pos.x, pos.y);
ctx.drawImage(img, -img_width / 2, -img_height / 2, img_width, img_height);
```

It fires events when the screen is updated as a rendering callback.

```javascript
window.addEventListener("view_changed", drawWidget);
```

-----

### TF Transforms

```javascript
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let tf = tfModule.tf;
```

An implementation of the TF graph that receives grouped TF data from the `/vizanti/tf_consolidated` topic.

Example:

```javascript
// Global rendering frame, as selected in global options
tf.fixed_frame;

// absoluteTransforms provides all known frames transformed to the global fixed frame for quick rendering
const absolute_frame = tf.absoluteTransforms["base_link"];
absolute_frame.translation; // Vector3
absolute_frame.rotation;    // Quaternion

// Getting the yaw in radians for top-down rendering
let yaw = absolute_frame.rotation.toEuler().h;

// transforms provides the relative parent-child transforms as given by TF
const relative_frame = tf.transforms["base_link"];
relative_frame.parent; // "odom", most likely
```

Like the view module, it fires events when the TF state is updated.

```javascript
window.addEventListener("tf_changed", drawWidget);
```

-----

### Vizanti Bridge

```javascript
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let rosbridge = rosbridgeModule.rosbridge;
```

This module provides the browser-side interface for ROS 2 communication.

The public widget-facing API keeps the familiar ROSLIB-style topic interface, while the implementation uses the Vizanti Ostirion binary WebSocket bridge.

Example:

```javascript
// All topics
let topics = await rosbridge.get_all_topics();

// All topics of a specified type
let specific_topics = await rosbridge.get_topics("sensor_msgs/msg/LaserScan");

// Built-in function to get the topic name passed by the widget creator
let topic = getTopic("{uniqueID}");

let listener = new ROSLIB.Topic({
    ros: rosbridge.ros,
    name: topic,
    messageType: "sensor_msgs/msg/LaserScan",
    throttle_rate: 30
});

listener.subscribe((msg) => {
    console.log(msg);
});
```

Publishing selected supported messages can also use the compatibility topic interface:

```javascript
let cmd_vel = new ROSLIB.Topic({
    ros: rosbridge.ros,
    name: "/cmd_vel",
    messageType: "geometry_msgs/msg/Twist"
});

cmd_vel.publish(new ROSLIB.Message({
    linear: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
}));
```

When adding support for additional message types, update both the browser-side decoding/publishing logic and the server-side message handling path.

-----

### Settings

```javascript
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let settings = persistentModule.settings;
```

Provides a localStorage object for saving widget settings.

Example:

```javascript
// If there is saved data, fetch it upon module load
if (settings.hasOwnProperty("{uniqueID}")) {
	const loaded_data = settings["{uniqueID}"];
	topic = loaded_data.topic;
	something = loaded_data.something;
	checkbox.checked = loaded_data.checkbox;
}

function saveSettings() {
	settings["{uniqueID}"] = {
		topic: topic,
		something: input.value,
		checkbox: checkbox.checked
	};
	settings.save();
}

// On input change
saveSettings();
```

-----

### Util

```javascript
let utilModule = await import(`${base_url}/js/modules/util.js`);
let imageToDataURL = utilModule.imageToDataURL;
```

The util module provides utility functions, including `imageToDataURL` for persistent image loading that does not trigger new server requests when changing an image `src` parameter.

```javascript
const persistent_image = await imageToDataURL("assets/image.svg");
```

-----

### Status

```javascript
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let Status = StatusModule.Status;
```

A helper class that emulates the RViz-style status indicator for each widget.

First, add this element to the widget modal. The current convention is directly below the title text.

```html
<p id="{uniqueID}_status" class="status">Status: Ok.</p>
```

Then instantiate the class and pass it the icon and status elements:

```javascript
// By default the status is "Ok" as defined in the HTML
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

// Show an error
status.setError("Empty topic.");

// Show a warning
status.setWarn("No data received.");

// Everything should be working fine
status.setOK();
```

-----

### Joysticks

```javascript
let joystickModule = await import(`${base_url}/js/modules/joystick.js`);
let nipplejs = joystickModule.nipplejs;
```

Joystick.js is a module wrapper for nipple.js.

Example usage:

[teleop_script.js](vizanti_server/public/templates/teleop/teleop_script.js)

-----

### Satellite Tiles

```javascript
let navsatModule = await import(`${base_url}/js/modules/navsat.js`);
let navsat = navsatModule.navsat;
```

A module for downloading, storing, and loading slippy map tiles, mainly for use in:

[satelite_script.js](vizanti_server/public/templates/satelite/satelite_script.js)

Example:

```javascript
const tileURL = "https://tile.openstreetmap.org/19/123/254.png";

let tileImage = navsat.live_cache[tileURL];

// Could be a placeholder if the tile is still loading, or not downloaded yet
if (!tileImage || !tileImage.complete) {
    tileImage = placeholder;
    navsat.enqueue(tileURL);
}

// Latitude and longitude to tile indices
let tile_coords = navsat.coordToTile(longitude, latitude, zoomLevel);

// Reverse conversion, gets the bottom-left corner of the tile
let tile_fix = navsat.tileToCoord(x, y, zoomLevel);

// Distance between two points on a sphere
let distance_meters = navsat.haversine(latitude_a, longitude_a, latitude_b, longitude_b);
```

More general info:

https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames

https://github.com/nobleo/rviz_satellite

-----

## Contribution Notes

Keep widget code modular and localized to its template folder when possible.

Keep server-side operations deterministic and avoid shell command construction. Use structured argument lists for subprocess calls.

Validate file paths, topic names, node names, and package names before passing user-provided values into ROS 2 tooling.

When adding new browser-supported ROS 2 message types, document the type and test the decoder with real serialized ROS 2 data.

When changing the WebSocket transport, keep browser and server opcode handling synchronized.

When adding robot-control features, keep the command path explicit and document which ROS 2 topic or action is used.
