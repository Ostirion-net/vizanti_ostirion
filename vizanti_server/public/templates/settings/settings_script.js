let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let view = viewModule.view;
let tf = tfModule.tf;
let settings = persistentModule.settings;
let saveJsonToFile = persistentModule.saveJsonToFile;
let Status = StatusModule.Status;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const selectionbox = document.getElementById("{uniqueID}_fixedframe");
const colourpicker = document.getElementById("{uniqueID}_colorpicker");

colourpicker.addEventListener("input", (event) =>{
	document.body.style.backgroundColor = colourpicker.value;
	saveSettings();
});

// Settings
if (settings.hasOwnProperty('{uniqueID}')) {
	const loadedData = settings['{uniqueID}'];
	tf.setFixedFrame(loadedData.fixed_frame);
	document.body.style.backgroundColor = loadedData.background_color;
}else{
	if(tf.fixed_frame == ""){
		tf.setFixedFrame("odom");
		status.setWarn("No frame selected, defaulting to odom");
	}
	saveSettings();
}

function saveSettings() {
	settings['{uniqueID}'] = {
		fixed_frame: tf.fixed_frame,
		background_color: colourpicker.value
	};
	settings.save();
}

// TF frame list
function setFrameList(){

	let framelist = "";
	for (const key of tf.frame_list.values()) {
		framelist += "<option value='"+key+"'>"+key+"</option>"
	}
	selectionbox.innerHTML = framelist;

	if(tf.frame_list.has(tf.fixed_frame)){
		selectionbox.value = tf.fixed_frame;
	}else{
		framelist += "<option value='"+tf.fixed_frame+"'>"+tf.fixed_frame+"</option>"
		selectionbox.innerHTML = framelist
		selectionbox.value = tf.fixed_frame;
	}
}

selectionbox.addEventListener("change", (event) => {
	tf.setFixedFrame(selectionbox.value);
	status.setOK();
	saveSettings();
});

selectionbox.addEventListener("click", setFrameList);
icon.addEventListener("click", setFrameList);

// Buttons
const modal = document.getElementById("{uniqueID}_modal");

document.getElementById("{uniqueID}_reset_view").addEventListener("click", (event) =>{
	view.reset();
	modal.style.display = "none";
});

document.getElementById("{uniqueID}_delete_persistent").addEventListener("click", async (event) =>{
	status.setWarn("Are you sure about that?");
	let del_ok = await confirm("Are you sure you want to delete your saved widget setup? This will refresh the page.");

	if(del_ok){
		localStorage.removeItem("settings");
		window.location.reload();
	}

	status.setOK();
});

document.getElementById("{uniqueID}_export_persistent").addEventListener("click", async (event) =>{
	if(await confirm("Save current layout to server? This will overwrite the default server layout.")){
		try {
			const response = await fetch('save_config', {
				method: 'POST',
				headers: { 'Content-Type': 'application/json' },
				body: JSON.stringify(settings)
			});
			if(response.ok) {
				alert("Layout successfully saved to the server.");
			} else {
				alert("Failed to save layout to server. Check terminal logs.");
			}
		} catch (error) {
			console.error('Error saving config:', error);
			alert("Network error while saving layout.");
		}
		modal.style.display = "none";
	}
});

document.getElementById("{uniqueID}_import_persistent").addEventListener("click", async (event) =>{
	if(await confirm("Load layout from server? This will overwrite your current local layout.")){
		try {
			const response = await fetch('load_config');
			if(response.ok) {
				const data = await response.text();
				settings.fromJSON(data);
				settings.save();
				location.reload(false);
			} else {
				alert("Failed to load layout from server.");
			}
		} catch (error) {
			console.error('Error loading config:', error);
			alert("Network error while loading layout.");
		}
		modal.style.display = "none";
	}
});


console.log("Settings Widget Loaded {uniqueID}")
