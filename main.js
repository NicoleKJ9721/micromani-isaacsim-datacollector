"use strict";
const electron = require("electron");
const path = require("path");
const fs = require("fs");
const { app } = require("electron");
const isMac = process.platform === "darwin";
const template = [
  ...isMac ? [{
    label: app.getName(),
    submenu: [
      { type: "separator" },
      { role: "services" },
      { type: "separator" },
      { role: "hide" },
      { role: "hideOthers" },
      { role: "unhide" },
      { type: "separator" },
      { role: "quit" }
    ]
  }] : [],
  {
    label: "Edit",
    submenu: [
      { role: "undo" },
      { role: "redo" },
      { type: "separator" },
      { role: "cut" },
      { role: "copy" },
      { role: "paste" },
      ...isMac ? [
        { role: "pasteAndMatchStyle" },
        { role: "delete" },
        { role: "selectAll" },
        { type: "separator" },
        {
          label: "Speech",
          submenu: [
            { role: "startSpeaking" },
            { role: "stopSpeaking" }
          ]
        }
      ] : [
        { role: "delete" },
        { type: "separator" },
        { role: "selectAll" }
      ]
    ]
  },
  ...isMac ? [{
    label: "View",
    submenu: [
      {
        role: "reload",
        accelerator: "Cmd+R"
      },
      { role: "togglefullscreen" }
    ]
  }] : [{
    label: "View",
    submenu: [
      {
        role: "reload",
        accelerator: "F5"
      },
      { role: "togglefullscreen" }
    ]
  }],
  {
    label: "About",
    submenu: [
      {
        label: "Version: " + app.getVersion()
      }
    ]
  }
];
if (process.platform === "darwin") {
  electron.app.setName("Isaac Sim WebRTC Streaming Client");
}
const getWindowStatePath = () => {
  const userDataPath = electron.app.getPath("userData");
  return path.join(userDataPath, "window-state.json");
};
const loadWindowState = () => {
  try {
    const statePath = getWindowStatePath();
    if (fs.existsSync(statePath)) {
      const state = JSON.parse(fs.readFileSync(statePath, "utf8"));
      return {
        x: state.x,
        y: state.y,
        width: state.width,
        height: state.height,
        isMaximized: state.isMaximized
      };
    }
  } catch (error) {
    console.error("Error loading window state:", error);
  }
  return {
    x: void 0,
    y: void 0,
    width: 800,
    height: 600,
    isMaximized: false
  };
};
const saveWindowState = (window) => {
  try {
    const state = {
      x: window.getPosition()[0],
      y: window.getPosition()[1],
      width: window.getSize()[0],
      height: window.getSize()[1],
      isMaximized: window.isMaximized()
    };
    const statePath = getWindowStatePath();
    fs.writeFileSync(statePath, JSON.stringify(state));
  } catch (error) {
    console.error("Error saving window state:", error);
  }
};
const createWindow = () => {
  const windowState = loadWindowState();
  const mainWindow = new electron.BrowserWindow({
    x: windowState.x,
    y: windowState.y,
    width: windowState.width,
    height: windowState.height,
    title: "Isaac Sim WebRTC Streaming Client",
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: true,
      webSecurity: true,
      allowRunningInsecureContent: false,
      spellcheck: true,
      enableWebSQL: true,
      enableBlinkFeatures: "ClipboardReadWrite,ClipboardEvents"
    }
  });
  if (windowState.isMaximized) {
    mainWindow.maximize();
  }
  mainWindow.on("close", () => {
    saveWindowState(mainWindow);
  });
  try {
    if (false) ;
    else {
      const indexPath = path.join(__dirname, "renderer", "index.html");
      console.log("Loading from:", indexPath);
      mainWindow.loadFile(indexPath);
    }
  } catch (error) {
    console.error("Failed to load application:", error);
    electron.app.quit();
  }
};
const menu = electron.Menu.buildFromTemplate(template);
electron.app.on("ready", () => {
  electron.Menu.setApplicationMenu(menu);
  createWindow();
});
electron.app.on("window-all-closed", () => {
  if (process.platform !== "darwin") {
    electron.app.quit();
  }
});
electron.app.on("activate", () => {
  if (electron.BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});
//# sourceMappingURL=main.js.map
