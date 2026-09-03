// PiSlider Retime — DaVinci Resolve Workflow Integration
// Starts retime_server.py then loads the panel UI in an Electron window.

const { app, BrowserWindow } = require('electron');
const http   = require('http');
const path   = require('path');
const { spawn } = require('child_process');
const WorkflowIntegration = require('./WorkflowIntegration.node');

const PLUGIN_ID   = 'com.pislider.retime';
const PORT        = 9077;

// ── Path to retime_server.py ─────────────────────────────────────────────────
// Update this if you move the project folder.
const SERVER_SCRIPT = path.join(
    require('os').homedir(),
    'Documents', 'slider claud code', 'retime_server.py'
);

let mainWindow = null;


// ── Server helpers ────────────────────────────────────────────────────────────

function checkServer() {
    return new Promise(resolve => {
        http.get(`http://localhost:${PORT}/status`, res => {
            resolve(res.statusCode === 200);
        }).on('error', () => resolve(false));
    });
}

async function ensureServer() {
    if (await checkServer()) return;

    const proc = spawn('python3', [SERVER_SCRIPT], {
        detached: true,
        stdio:    'ignore',
    });
    proc.unref();

    // Poll until the server responds (up to 8 s)
    for (let i = 0; i < 16; i++) {
        await new Promise(r => setTimeout(r, 500));
        if (await checkServer()) break;
    }
}


// ── Window ────────────────────────────────────────────────────────────────────

function createWindow() {
    mainWindow = new BrowserWindow({
        width:  460,
        height: 720,
        title:  'PiSlider Retime',
        webPreferences: {
            preload:          path.join(__dirname, 'preload.js'),
            contextIsolation: true,
        },
    });

    // Load panel UI from our local server
    mainWindow.loadURL(`http://localhost:${PORT}`);
    mainWindow.on('closed', () => { mainWindow = null; });
}


// ── App lifecycle ─────────────────────────────────────────────────────────────

app.whenReady().then(async () => {
    await WorkflowIntegration.Initialize(PLUGIN_ID);
    await ensureServer();
    createWindow();
});

app.on('window-all-closed', () => {
    WorkflowIntegration.CleanUp();
    if (process.platform !== 'darwin') app.quit();
});

app.on('activate', () => {
    if (mainWindow === null) createWindow();
});
