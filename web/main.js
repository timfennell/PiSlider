/**
 * main.js — PiSlider Web Command Engine v2.4
 *
 * New in v2.4:
 * - Session persistence: reconnect restores full state from server
 * - Stop/Reset dual-mode button
 * - HG locks exposure controls when enabled
 * - Trigger mode: picam_motion_only / picam_motion_hybrid + ROI settings
 * - Sequence Progress: estimated end time, remaining time, live interval
 * - Removed Generate Plan button (runs automatically)
 * - init packet from server restores all UI fields on reconnect
 */

"use strict";

// ─── CONFIG ──────────────────────────────────────────────────────────────────
const WS_URL = `ws://${window.location.host}/ws`;
const RECONNECT_DELAY = 2000;

// Quarter-stop shutter table (seconds)
const SHUTTER_VALS = buildQuarterStops([
    1 / 8000, 1 / 4000, 1 / 2000, 1 / 1000, 1 / 500, 1 / 250, 1 / 125,
    1 / 60, 1 / 30, 1 / 15, 1 / 8, 1 / 4, 1 / 2,
    1, 2, 4, 8, 15, 30
]);

// Quarter-stop ISO table
const ISO_VALS = buildQuarterStops([
    100, 200, 400, 800, 1600, 3200, 6400, 12800
]);

// ─── STATE ───────────────────────────────────────────────────────────────────
let socket = null;
let joystickActive = false;
let isRunning = false;          // tracks sequence state — kept in sync via run_state msgs
let latestFrameInterval = null;           // setInterval handle for latest-frame polling during a run
let currentMode = 'timelapse';
let _loupeUserVisible = true;           // controlled by 🔍 toggle button
let relay1On = false;
let relay2On = false;
let motionScripts = [];
let folderBrowserCurrentPath = '/home/tim/Pictures';

// ─── DOM CACHE ───────────────────────────────────────────────────────────────
const els = {
    mjpegFeed: document.getElementById('mjpegFeed'),
    latestFrame: document.getElementById('latestFrame'),
    feedContainer: document.getElementById('feedContainer'),
    statusLine: document.getElementById('statusLine'),
    debugOverlay: document.getElementById('debugOverlay'),
    shutterIndicator: document.getElementById('shutter_indicator'),

    joystickPad: document.getElementById('joystickPad'),
    joystickKnob: document.getElementById('joystickKnob'),

    cameraSelect: document.getElementById('camera_select'),
    sonyGuide: document.getElementById('sony_guide'),
    savePath: document.getElementById('save_path'),

    // Exposure
    aeToggle: document.getElementById('ae_toggle'),
    awbToggle: document.getElementById('awb_toggle'),
    shutterSlider: document.getElementById('shutter_slider'),
    shutterLabel: document.getElementById('shutter_label'),
    isoSlider: document.getElementById('iso_slider'),
    isoLabel: document.getElementById('iso_label'),
    wbSlider: document.getElementById('wb_slider'),
    wbLabel: document.getElementById('wb_label'),

    // Sequence
    totalFrames: document.getElementById('total_frames'),
    vibeDelay: document.getElementById('vibe_delay'),
    expMargin: document.getElementById('exp_margin'),

    // Telemetry
    nodeReadout: document.getElementById('nodeReadout'),
    curFrame: document.getElementById('cur_f'),
    totFrame: document.getElementById('tot_f'),
    progressMsg: document.getElementById('progress_msg'),
    valS: document.getElementById('val_s'),
    valP: document.getElementById('val_p'),
    valT: document.getElementById('val_t'),

    // HG telemetry
    hgPhase: document.getElementById('hg_phase'),
    hgSunAlt: document.getElementById('hg_sun_alt'),
    hgEV: document.getElementById('hg_ev'),
    hgISO: document.getElementById('hg_iso'),
    hgShutter: document.getElementById('hg_shutter'),
    hgKelvin: document.getElementById('hg_kelvin'),

    // Motion script
    scriptSelect: document.getElementById('motion_script_select'),

    // Fan — auto-managed, no slider in UI
    fanSlider: null,
    fanPct: null,

    // Macro panel
    macroPanel: document.getElementById('macro_panel'),
};

// ─── INIT ────────────────────────────────────────────────────────────────────
window.onload = () => {
    log("Interface bootstrapped. Initialising hardware link…");
    initSliders();
    connectWS();
    setupJoystick();
    setupSliderStrip();
    setupCanvasOverlay();
    setupSeqMode();
    onCameraChange(els.cameraSelect?.value || 'picam', true);
    loadMotionScripts();
    updateHGExposureLock();
    onTriggerModeChange('normal', false);
    document.body.classList.add('idle');
    refreshDiskInfo();
    startLoupePolling();
    scanDrives();
    macroCalc();
    sendCmd('macro_load_lens_profiles');
    // Silently attempt GPS on load — updates HG lat/lon if browser permits
    grabGPS(true);
    // Auto-reconnect MJPEG feed if the stream drops (e.g. during a cinematic move)
    _setupFeedWatchdog();
};

function _setupFeedWatchdog() {
    if (!els.mjpegFeed) return;
    let _feedWatchdogTimer = null;

    const reconnect = () => {
        if (els.mjpegFeed.style.display === 'none') return;
        els.mjpegFeed.src = `/video_feed?t=${Date.now()}`;
    };

    // Reconnect immediately on any stream error
    els.mjpegFeed.addEventListener('error', () => {
        clearTimeout(_feedWatchdogTimer);
        _feedWatchdogTimer = setTimeout(reconnect, 800);
    });

    // Stale-feed watchdog: if the image hasn't updated for 8s while visible,
    // force a reconnect. Picamera2 streams ~20fps so 8s = ~160 missed frames.
    let _lastLoad = Date.now();
    els.mjpegFeed.addEventListener('load', () => { _lastLoad = Date.now(); });

    setInterval(() => {
        if (els.mjpegFeed.style.display === 'none') return;
        if (Date.now() - _lastLoad > 8000) {
            reconnect();
            _lastLoad = Date.now();   // reset so we don't immediately fire again
        }
    }, 4000);
}

// ─── QUARTER-STOP MATH ───────────────────────────────────────────────────────
function buildQuarterStops(stops) {
    const out = [];
    for (let i = 0; i < stops.length - 1; i++) {
        const s = stops[i], e = stops[i + 1];
        for (let q = 0; q < 4; q++) {
            out.push(s * Math.pow(e / s, q / 4));
        }
    }
    out.push(stops[stops.length - 1]);
    return out;
}

function prettyShutter(s) {
    if (s >= 1) return `${Math.round(s * 10) / 10}s`;
    return `1/${Math.round(1 / s)}`;
}

function initSliders() {
    // Set initial slider positions to sensible defaults
    const shutterDefault = SHUTTER_VALS.findIndex(v => Math.abs(v - 1 / 125) < 0.0001);
    els.shutterSlider.max = SHUTTER_VALS.length - 1;
    els.shutterSlider.value = shutterDefault >= 0 ? shutterDefault : Math.floor(SHUTTER_VALS.length / 2);
    els.shutterLabel.innerText = prettyShutter(SHUTTER_VALS[els.shutterSlider.value]);

    const isoDefault = ISO_VALS.findIndex(v => Math.abs(v - 400) < 1);
    els.isoSlider.max = ISO_VALS.length - 1;
    els.isoSlider.value = isoDefault >= 0 ? isoDefault : Math.floor(ISO_VALS.length / 2);
    els.isoLabel.innerText = Math.round(ISO_VALS[els.isoSlider.value]);
}

// ─── SLIDER CALLBACKS ────────────────────────────────────────────────────────
function onShutterSlider(val) {
    const s = SHUTTER_VALS[parseInt(val)];
    els.shutterLabel.innerText = prettyShutter(s);
    // Touching shutter disables AE
    if (els.aeToggle.checked) {
        els.aeToggle.checked = false;
    }
    sendPicamSettings();
}

function onISOSlider(val) {
    const iso = Math.round(ISO_VALS[parseInt(val)]);
    els.isoLabel.innerText = iso;
    if (els.aeToggle.checked) {
        els.aeToggle.checked = false;
    }
    sendPicamSettings();
}

function onWBSlider(val) {
    els.wbLabel.innerText = `${val}K`;
    if (els.awbToggle.checked) {
        els.awbToggle.checked = false;
    }
    sendPicamSettings();
}

function sendPicamSettings() {
    const s = SHUTTER_VALS[parseInt(els.shutterSlider.value)];
    const iso = Math.round(ISO_VALS[parseInt(els.isoSlider.value)]);
    const kelvin = parseInt(els.wbSlider.value);
    sendCmd('set_picam_settings', {
        ae: els.aeToggle.checked,
        awb: els.awbToggle.checked,
        shutter_s: s,
        iso: iso,
        kelvin: kelvin,
    });
}

// ─── HOLY GRAIL NIGHT PRESETS ────────────────────────────────────────────────
// Each preset sets ev_night + the hardware limits that match the scene type.
// ev_night is in pixel-EV space: lower = darker target = longer/higher exposure.
// The cold start always begins at shutter_max_night + iso_max and corrects down.
const HG_NIGHT_PRESETS = {
    dark_sky: {
        ev_night: 3.0, iso_max: 6400, shutter_max_night: 25, interval_night: 30,
        kelvin_night: 3800,
        note: 'Stars / Milky Way: maximum exposure. Astro model auto-adjusts if the moon rises during the sequence.',
    },
    moonlit: {
        ev_night: 8.0, iso_max: 3200, shutter_max_night: 15, interval_night: 15,
        kelvin_night: 4200,
        note: 'Moon-lit: foreground clearly visible. Astro model tracks moon altitude and phase — bright full moon = shorter exposures.',
    },
    landscape: {
        ev_night: 5.0, iso_max: 6400, shutter_max_night: 20, interval_night: 25,
        kelvin_night: 4000,
        note: 'Landscape without moon: some foreground detail, longer exposures than moonlit. Good starting point for light-polluted horizons.',
    },
    urban: {
        ev_night: 10.5, iso_max: 1600, shutter_max_night: 8, interval_night: 10,
        kelvin_night: 3200,
        note: 'City / artificial light: bright scene, short exposures, highlight control active. Astro model has less influence — city is its own light source.',
    },
};

let _activeNightPreset = null;

function applyNightPreset(name) {
    const p = HG_NIGHT_PRESETS[name];
    if (!p) return;
    const _s = (id, v) => { const el = document.getElementById(id); if (el) el.value = v; };
    _s('hg_ev_night',          p.ev_night);
    _s('hg_iso_max',           p.iso_max);
    _s('hg_shutter_max_night', p.shutter_max_night);
    _s('hg_int_night',         p.interval_night);
    _s('hg_k_night',           p.kelvin_night);

    // Highlight selected card, clear others
    _activeNightPreset = name;
    ['dark_sky', 'moonlit', 'landscape', 'urban'].forEach(k => {
        const el = document.getElementById(`preset_${k}`);
        if (el) el.style.borderColor = (k === name) ? 'var(--accent-teal)' : 'transparent';
    });

    // Show preset note
    const noteEl = document.getElementById('night_preset_note');
    if (noteEl) { noteEl.textContent = p.note; noteEl.style.display = ''; }

    updateEvNightHint();
    sendHGSettings();
}

function updateEvNightHint() {
    const ev         = parseFloat(document.getElementById('hg_ev_night')?.value) || 3.0;
    const isoMax     = parseInt(document.getElementById('hg_iso_max')?.value)    || 3200;
    const shutterMax = parseFloat(document.getElementById('hg_shutter_max_night')?.value) || 25;
    const hint       = document.getElementById('ev_night_hint');
    if (!hint) return;

    let desc;
    if      (ev <= 4.0)  desc = 'Near-black target — maximum exposure, faint stars';
    else if (ev <= 6.5)  desc = 'Dim scene — faint foreground detail, long exposures';
    else if (ev <= 9.0)  desc = 'Moonlit — foreground visible, moderate exposure';
    else if (ev <= 11.0) desc = 'Bright night — city / artificial light';
    else                 desc = 'Very bright — strong artificial or near-dawn lighting';

    hint.textContent = `${desc}. Cold start: ISO ${isoMax} / ${shutterMax}s.`;
}

// ─── HOLY GRAIL SETTINGS ─────────────────────────────────────────────────────
function sendHGSettings() {
    const basic = {
        enabled: document.getElementById('hg_enabled').checked,
        lat: parseFloat(document.getElementById('hg_lat').value),
        lon: parseFloat(document.getElementById('hg_lon').value),
        tz: document.getElementById('hg_tz').value.trim(),
        cam_az: parseFloat(document.getElementById('hg_cam_az').value),
        cam_alt: parseFloat(document.getElementById('hg_cam_alt').value),
        hfov: parseFloat(document.getElementById('hg_hfov').value),
        vfov: parseFloat(document.getElementById('hg_vfov').value),
        // Advanced
        ev_day: parseFloat(document.getElementById('hg_ev_day').value),
        ev_golden: parseFloat(document.getElementById('hg_ev_golden').value),
        ev_twilight: parseFloat(document.getElementById('hg_ev_twilight').value),
        ev_night: parseFloat(document.getElementById('hg_ev_night').value),
        kelvin_day: parseInt(document.getElementById('hg_k_day').value),
        kelvin_golden: parseInt(document.getElementById('hg_k_golden').value),
        kelvin_twilight: parseInt(document.getElementById('hg_k_twilight').value),
        kelvin_night: parseInt(document.getElementById('hg_k_night').value),
        interval_day: parseFloat(document.getElementById('hg_int_day').value),
        interval_golden: parseFloat(document.getElementById('hg_int_golden').value),
        interval_twilight: parseFloat(document.getElementById('hg_int_twilight').value),
        interval_night: parseFloat(document.getElementById('hg_int_night').value),
        iso_min: parseInt(document.getElementById('hg_iso_min').value),
        iso_max: parseInt(document.getElementById('hg_iso_max').value),
        aperture_day: parseFloat(document.getElementById('hg_ap_day').value),
        aperture_night: parseFloat(document.getElementById('hg_ap_night').value),
        shutter_max_twilight: parseFloat(document.getElementById('hg_shutter_max_twilight').value),
        shutter_max_night: parseFloat(document.getElementById('hg_shutter_max_night').value),
    };
    sendCmd('set_hg_settings', basic);
    log(`HG Settings applied. Enabled=${basic.enabled}, Lat=${basic.lat}`);
}

// ─── XMP PERSPECTIVE CORRECTION ──────────────────────────────────────────────

function readLensData() {
    log('🔭 Reading lens data from last shot EXIF…');
    sendCmd('read_lens_data');
}

function _applyLensInfo(data) {
    const { focal_mm, lens_model, hfov, vfov, source } = data;

    // Fill HFOV / VFOV in HG panel
    const hfovEl = document.getElementById('hg_hfov');
    const vfovEl = document.getElementById('hg_vfov');
    if (hfovEl) hfovEl.value = hfov;
    if (vfovEl) vfovEl.value = vfov;

    // Update the readout label under the FOV fields
    const readout = document.getElementById('lens_info_readout');
    if (readout) {
        const label = lens_model && lens_model !== '----'
            ? `${lens_model} (${focal_mm.toFixed(0)}mm)`
            : `${focal_mm.toFixed(0)}mm`;
        readout.style.display = '';
        readout.textContent   = `📷 ${label} · HFOV ${hfov}° · VFOV ${vfov}°`;
    }

    // Push updated HFOV/VFOV to server
    sendHGSettings();

    const src = source === 'hg_calibration_usb' ? 'HG calibration shot' : 'last shot';
    const lensLabel = lens_model && lens_model !== '----' ? `${lens_model} — ` : '';
    log(`🔭 Lens from ${src}: ${lensLabel}${focal_mm.toFixed(0)}mm → HFOV ${hfov}° VFOV ${vfov}°`);
}

function _showFocalDetectedBanner(focalMm, lensModel) {
    // Remove any existing banner first
    const old = document.getElementById('focalDetectedBanner');
    if (old) old.remove();

    const label = lensModel && lensModel !== '----'
        ? `${lensModel} (${focalMm.toFixed(0)}mm)`
        : `${focalMm.toFixed(0)}mm`;

    const banner = document.createElement('div');
    banner.id = 'focalDetectedBanner';
    banner.style.cssText = `
        position:fixed; bottom:44px; left:50%; transform:translateX(-50%);
        background:#1a2a1a; border:1px solid var(--accent-green);
        border-radius:8px; padding:10px 14px; z-index:300;
        font-size:0.78rem; color:var(--accent-green);
        display:flex; align-items:center; gap:10px; max-width:90vw;
        box-shadow:0 4px 16px rgba(0,0,0,0.6);
    `;
    banner.innerHTML = `
        <span>🔍 Lens: <b>${label}</b> detected from EXIF</span>
        <button onclick="applyDetectedFocal(${focalMm})" style="
            background:var(--accent-green); color:#000; border:none;
            border-radius:4px; padding:4px 10px; font-size:0.75rem;
            cursor:pointer; white-space:nowrap; font-weight:600;">
            Use ${focalMm.toFixed(0)}mm
        </button>
        <button onclick="this.closest('#focalDetectedBanner').remove()" style="
            background:none; border:none; color:var(--text-dim);
            font-size:1rem; cursor:pointer; padding:0 4px; line-height:1;">✕</button>
    `;
    document.body.appendChild(banner);
    // Auto-dismiss after 20 seconds
    setTimeout(() => { if (banner.parentNode) banner.remove(); }, 20000);
}

function applyDetectedFocal(focalMm) {
    log(`📐 Focal length ${focalMm}mm applied to HG field of view`);
    const banner = document.getElementById('focalDetectedBanner');
    if (banner) banner.remove();
}

// ─── GPS AUTO-LOCATION ────────────────────────────────────────────────────────
async function grabGPS(silent = false) {
    // Use server-side location lookup (/api/gps) instead of browser geolocation.
    // navigator.geolocation is blocked by Chrome on non-HTTPS origins (http://pislider.local).
    // The Pi fetches its approximate location via ip-api.com — no browser HTTPS needed.
    const btn = document.getElementById('gpsBtn');
    if (btn) { btn.innerText = '⏳'; btn.style.color = 'var(--accent-gold)'; }
    try {
        const res = await fetch('/api/gps');
        const data = await res.json();
        if (data.error && !data.lat) {
            if (!silent) log(`⚠ GPS lookup failed: ${data.error} — enter coordinates manually.`);
            if (btn) { btn.innerText = '📍'; btn.style.color = '#555'; }
            return;
        }
        const latEl = document.getElementById('hg_lat');
        const lonEl = document.getElementById('hg_lon');
        if (latEl) latEl.value = data.lat;
        if (lonEl) lonEl.value = data.lon;
        // Also update timezone if returned
        const tzEl = document.getElementById('hg_timezone');
        if (tzEl && data.timezone) tzEl.value = data.timezone;
        sendHGSettings();
        if (btn) { btn.innerText = '📍'; btn.style.color = 'var(--accent-green)'; }
        const city = data.city ? ` (${data.city})` : '';
        log(`📍 Location updated: ${data.lat}, ${data.lon}${city}`);
        setTimeout(() => { if (btn) btn.style.color = '#888'; }, 3000);
    } catch (e) {
        if (!silent) log(`⚠ GPS: ${e.message} — enter coordinates manually.`);
        if (btn) { btn.innerText = '📍'; btn.style.color = '#555'; }
    }
}

// ─── RELAY CONTROL ───────────────────────────────────────────────────────────
function toggleRelay(n) {
    if (n === 1) {
        relay1On = !relay1On;
        document.getElementById('relay1_state').innerText = relay1On ? 'ON' : 'OFF';
        document.getElementById('relay1_btn').classList.toggle('relay-active', relay1On);
        sendCmd('set_relay', { relay: 1, on: relay1On });
    } else {
        relay2On = !relay2On;
        document.getElementById('relay2_state').innerText = relay2On ? 'ON' : 'OFF';
        document.getElementById('relay2_btn').classList.toggle('relay-active', relay2On);
        sendCmd('set_relay', { relay: 2, on: relay2On });
    }
}

// ─── FAN CONTROL ─────────────────────────────────────────────────────────────
function setFan(val) {
    if (els.fanPct) els.fanPct.innerText = val;
    sendCmd('set_fan', parseInt(val));
}

// ─── MOTION SCRIPTS ──────────────────────────────────────────────────────────
async function loadMotionScripts() {
    try {
        const resp = await fetch('/static/motion_scripts.json');
        const data = await resp.json();
        motionScripts = data.scripts || [];
        const sel = els.scriptSelect;
        motionScripts.forEach(s => {
            const opt = document.createElement('option');
            opt.value = s.id;
            opt.textContent = `${s.label} (${s.duration_s}s)`;
            sel.appendChild(opt);
        });
    } catch (e) {
        log(`Motion scripts not loaded: ${e}`);
    }
}

function loadMotionScript() {
    const id = els.scriptSelect.value;
    if (!id) return;
    sendCmd('load_motion_script', { script_id: id });
    log(`Motion script loaded: ${id}`);
}

// ─── COLLAPSIBLE PANELS ──────────────────────────────────────────────────────
function toggleAdvanced(id) {
    const panel = document.getElementById(id);
    const btn = panel.previousElementSibling;
    const open = panel.style.display !== 'none';
    panel.style.display = open ? 'none' : 'block';
    btn.textContent = open ? 'Advanced Settings ▼' : 'Advanced Settings ▲';
}

function toggleHomePanel(id) {
    const panel = document.getElementById(id);
    const btn = panel.previousElementSibling;
    // collapsible-content starts as display:none (CSS default), so empty string = closed
    const open = panel.style.display === 'block';
    panel.style.display = open ? 'none' : 'block';
    btn.innerHTML = open ? '⌂ Home Calibration ▼' : '⌂ Home Calibration ▲';
}

function toggleRigSetup() {
    const content = document.getElementById('rig_setup_content');
    const arrow   = document.getElementById('rig_setup_arrow');
    const open    = content.style.display !== 'none';
    content.style.display = open ? 'none' : 'block';
    if (arrow) arrow.textContent = open ? '▼' : '▲';
}

function goHome() {
    // Universal "Go to Home" — reuses the macro_go_home backend command which
    // moves all three axes (pan, tilt, focus rail) to their zeroed home position.
    macroGoHome();
}

// ─── MODE SWITCHING ──────────────────────────────────────────────────────────

// _applyModeUI: update panels/buttons for a mode without sending any command.
// Used both by setMode (user click) and by the init-packet restore path.
function _applyModeUI(mode) {
    currentMode = mode;
    ['timelapse', 'cinematic', 'macro'].forEach(m => {
        document.getElementById(`mode${m.charAt(0).toUpperCase() + m.slice(1)}`)
            .classList.toggle('active', m === mode);
    });

    const isMacro = mode === 'macro';
    const isCinematic = mode === 'cinematic';

    // Panel visibility
    els.macroPanel.style.display = isMacro ? 'block' : 'none';
    const cp = document.getElementById('cinematic_panel');
    if (cp) cp.style.display = isCinematic ? 'block' : 'none';

    // Load easing curves when macro mode is activated
    if (isMacro) {
        requestMacroEasingCurves();
    }

    // hg_section is always visible (contains soft limits + invert, needed in all modes)
    // Only hide the HG-specific sub-sections in cinematic/macro modes
    const hgOnly1 = document.getElementById('hg_timelapse_only');
    const hgOnly2 = document.getElementById('hg_timelapse_only_2');
    const triggerSection = document.getElementById('trigger_section');
    const seqPanel = document.getElementById('sequence_settings_panel');
    if (hgOnly1) hgOnly1.style.display = (isMacro || isCinematic) ? 'none' : '';
    if (hgOnly2) hgOnly2.style.display = (isMacro || isCinematic) ? 'none' : '';
    if (triggerSection) triggerSection.style.display = (isMacro || isCinematic) ? 'none' : '';
    // Sequence Settings is timelapse-only — hide in cinematic and macro
    if (seqPanel) seqPanel.style.display = (isMacro || isCinematic) ? 'none' : '';

    // Rig Setup (hg-limit-cal) is always visible — shown in all modes.
    // The collapsible #rig_setup_content can be toggled by the user via toggleRigSetup().

    // Hide timelapse Start button in non-timelapse modes
    const startBtn = document.getElementById('startBtn');
    if (startBtn) startBtn.style.display = (isMacro || isCinematic) ? 'none' : '';

    // Record + Run header button: timelapse-only (cinematic has its own in-panel button)
    const rrBtn = document.getElementById('recordRunBtn');
    if (rrBtn) rrBtn.style.display = (isMacro || isCinematic) ? 'none' : '';

    // Enforce safe state when entering macro or cinematic mode
    if (isMacro || isCinematic) {
        const hgCb = document.getElementById('hg_enabled');
        if (hgCb && hgCb.checked) {
            hgCb.checked = false;
            sendHGSettings();
        }
        updateHGExposureLock();
        const normalRadio = document.querySelector('input[name="trigger_mode"][value="normal"]');
        if (isMacro && normalRadio && !normalRadio.checked) {
            normalRadio.checked = true;
            onTriggerModeChange('normal', true);
        }
    }

    // Switch feed container aspect ratio to match stream
    const container = document.getElementById('feedContainer');
    if (container) {
        container.style.aspectRatio = isCinematic ? '16 / 9' : '4 / 3';
    }

    // Always hide motion ROI box in cinematic and macro modes
    if (isCinematic || isMacro) {
        setRoiVisible(false);
    }

    // Hide Motion Path panel in Macro SCAN mode (only needed for Macro ART)
    const motionPathPanel = document.getElementById('motion_path_panel');
    if (motionPathPanel) {
        motionPathPanel.style.display = (isMacro && _macroMode === 'scan') ? 'none' : '';
    }

    // Always refresh the Motion Path panel so keyframes + library are current
    // (but it's hidden in macro scan mode).
    // Timelapse, cinematic, and macro art modes all share motion paths.
    sendCmd('cinematic_get_state');
    sendCmd('cinematic_list_moves');
    // Re-render path stats for the newly active mode
    _updatePathSummary();

    if (!isCinematic && _cineInertiaRunning) {
        sendCmd('cinematic_live_stop');
        _cineInertiaRunning = false;
    }
}

function setMode(mode) {
    _applyModeUI(mode);
    sendCmd('set_mode', { value: mode });
    log(`Mode: ${mode.toUpperCase()}`);
}

function openSessionGraph() {
    // Open macro or timelapse session graph based on current mode
    const url = (currentMode === 'macro') ? '/macro_graph' : '/graph';
    window.open(url, '_blank');
}

// ─── SEQUENCE MODE (frames vs start/end time) ─────────────────────────────────
function setupSeqMode() {
    // Seed datetime-local inputs with "now" and "now + 1hr"
    const now = new Date();
    const plus1 = new Date(now.getTime() + 3600000);
    const fmt = d => d.toISOString().slice(0, 16);
    const st = document.getElementById('seq_start_time');
    const et = document.getElementById('seq_end_time');
    if (st) st.value = fmt(now);
    if (et) et.value = fmt(plus1);

    // Wire duration calc
    ['seq_start_time', 'seq_end_time', 'hg_int_dur'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.addEventListener('change', calcDurationFrames);
    });
    calcDurationFrames();
}

function onSeqModeChange(mode) {
    document.getElementById('seq_frames_row').style.display = (mode === 'frames') ? 'flex' : 'none';
    document.getElementById('seq_duration_row').style.display = (mode === 'duration') ? 'block' : 'none';
    if (mode === 'duration') calcDurationFrames();
}

function calcDurationFrames() {
    const st = document.getElementById('seq_start_time')?.value;
    const et = document.getElementById('seq_end_time')?.value;
    const iv = parseFloat(document.getElementById('hg_int_dur')?.value) || 5;
    const out = document.getElementById('seq_duration_calc');
    if (!st || !et || !out) return;
    const secs = (new Date(et) - new Date(st)) / 1000;
    if (secs <= 0) { out.innerText = 'End must be after Start'; return; }
    const frames = Math.floor(secs / iv);
    out.innerText = `≈ ${frames} frames  (${(secs / 3600).toFixed(1)} hrs)`;
}

function getSeqConfig() {
    const seqMode = document.querySelector('input[name="seq_mode"]:checked')?.value || 'frames';
    const triggerMode = document.querySelector('input[name="trigger_mode"]:checked')?.value || 'normal';

    let total_frames, interval, save_path;
    save_path = document.getElementById('save_path')?.value || '/home/tim/Pictures/PiSlider';

    if (seqMode === 'frames') {
        // Use the manual interval field; HG overrides per-phase at runtime if enabled
        const rawInterval = document.getElementById('manual_interval')?.value;
        interval = parseFloat(rawInterval) || 5;
        total_frames = parseInt(document.getElementById('total_frames')?.value) || 300;
        const hgOn = document.getElementById('hg_enabled')?.checked;
        if (hgOn) {
            log(`⏱ Base interval = ${interval}s (HG ON — per-phase intervals override at runtime)`);
        } else {
            log(`⏱ Interval = ${interval}s`);
        }
    } else {
        interval = parseFloat(document.getElementById('hg_int_dur')?.value) || 5;
        const st = new Date(document.getElementById('seq_start_time')?.value);
        const et = new Date(document.getElementById('seq_end_time')?.value);
        const secs = (et - st) / 1000;
        total_frames = Math.max(1, Math.floor(secs / interval));
    }

    // For time-based mode, pass the schedule start time (raw datetime-local)
    // and the app's configured timezone so the server can interpret it correctly.
    let schedule_start = null;
    let schedule_tz    = null;
    if (seqMode === 'duration') {
        const stVal = document.getElementById('seq_start_time')?.value;
        if (stVal) {
            // Send the raw "YYYY-MM-DDTHH:MM" value — NOT converted to UTC.
            // The server will apply schedule_tz to get the correct local time.
            schedule_start = stVal;
            schedule_tz    = document.getElementById('hg_timezone')?.value
                             || 'America/Chicago';
        }
    }

    return {
        interval,
        total_frames,
        vibe_delay:    parseFloat(document.getElementById('vibe_delay')?.value) || 1.0,
        exp_margin:    parseFloat(document.getElementById('exp_margin')?.value) || 0.2,
        tl_preroll_s:  parseFloat(document.getElementById('tl_preroll')?.value)  || 0,
        save_path,
        trigger_mode: triggerMode,
        mode: currentMode,
        schedule_start,
        schedule_tz,
    };
}

// ─── SCHEDULED WAIT STATE ────────────────────────────────────────────────────
let _schedWaitInterval = null;   // setInterval handle for countdown tick

function setScheduledWaitState(secondsRemaining, targetTime) {
    // Don't enter the full "RUNNING" state — just show a waiting indicator.
    // Lock the Start button and show a countdown in the status line.
    const startBtn = document.getElementById('startBtn');
    const stopBtn  = document.getElementById('stopBtn');
    const statusEl = document.getElementById('statusLine');

    if (startBtn) {
        startBtn.innerText = 'WAITING…';
        startBtn.style.opacity = '0.4';
        startBtn.style.pointerEvents = 'none';
    }
    if (stopBtn) {
        stopBtn.innerText = 'Cancel';
        stopBtn.classList.add('stop-style');
        stopBtn.classList.remove('reset-style');
    }

    // Update the status line with a live countdown.
    // remaining=0 is used as a placeholder when reconnecting (real value
    // arrives shortly via a scheduled_wait message) — show "…" instead
    // of clearing immediately.
    let remaining = secondsRemaining;
    const _updateCountdown = () => {
        // remaining<0 means the countdown expired before the server confirmed
        // start — clear and let run_state:true take over.
        if (remaining < 0) {
            clearScheduledWaitState();
            return;
        }
        if (remaining === 0) {
            // Placeholder state — server will resend real value soon
            if (statusEl) {
                statusEl.innerText = `⏳ SCHEDULED — starts at ${targetTime || '…'}`;
                statusEl.style.color = 'var(--accent-gold)';
            }
            // Don't decrement below -1 in case it stays stuck
            remaining--;
            return;
        }
        const h = Math.floor(remaining / 3600);
        const m = Math.floor((remaining % 3600) / 60);
        const s = remaining % 60;
        const hms = h > 0
            ? `${h}h ${String(m).padStart(2,'0')}m`
            : m > 0
                ? `${m}m ${String(s).padStart(2,'0')}s`
                : `${s}s`;
        if (statusEl) {
            statusEl.innerText = `⏳ SCHEDULED — starts at ${targetTime} (in ${hms})`;
            statusEl.style.color = 'var(--accent-gold)';
        }
        remaining--;
    };
    _updateCountdown();
    if (_schedWaitInterval) clearInterval(_schedWaitInterval);
    _schedWaitInterval = setInterval(_updateCountdown, 1000);
}

function clearScheduledWaitState() {
    if (_schedWaitInterval) { clearInterval(_schedWaitInterval); _schedWaitInterval = null; }
    const startBtn = document.getElementById('startBtn');
    const stopBtn  = document.getElementById('stopBtn');
    if (startBtn) {
        startBtn.innerText = 'Start Sequence';
        startBtn.style.opacity = '';
        startBtn.style.pointerEvents = '';
    }
    if (stopBtn) {
        stopBtn.innerText = 'Reset Session';
        stopBtn.classList.remove('stop-style');
        stopBtn.classList.add('reset-style');
    }
    // Status line will be updated by the run_state message that follows
}

// ─── RUN STATE ────────────────────────────────────────────────────────────────
function setRunState(running) {
    // Clear any pending scheduled-wait countdown first
    if (_schedWaitInterval) { clearInterval(_schedWaitInterval); _schedWaitInterval = null; }

    isRunning = running;
    document.body.classList.toggle('running', running);
    document.body.classList.toggle('idle', !running);

    const startBtn = document.getElementById('startBtn');
    const stopBtn = document.getElementById('stopBtn');

    if (startBtn) {
        startBtn.innerText = running ? 'RUNNING…' : 'Start Sequence';
        startBtn.style.opacity = running ? '0.4' : '1';
        startBtn.style.pointerEvents = running ? 'none' : 'auto';
    }

    // Stop button becomes E-Stop when running, Reset when idle
    if (stopBtn) {
        if (running) {
            stopBtn.innerText = 'E-Stop';
            stopBtn.classList.add('stop-style');
            stopBtn.classList.remove('reset-style');
        } else {
            stopBtn.innerText = 'Reset Session';
            stopBtn.classList.remove('stop-style');
            stopBtn.classList.add('reset-style');
        }
    }

    // Disable all macro movement buttons during sequence (hands-off mode)
    const macroMovementBtns = document.querySelectorAll('.macroMovementBtn');
    const macroGoHomeBtn = document.getElementById('macroGoHomeBtn');
    macroMovementBtns.forEach(btn => {
        btn.disabled = running;
        btn.style.opacity = running ? '0.4' : '1';
        btn.style.pointerEvents = running ? 'none' : 'auto';
    });
    if (macroGoHomeBtn) {
        macroGoHomeBtn.disabled = running;
        macroGoHomeBtn.style.opacity = running ? '0.4' : '1';
        macroGoHomeBtn.style.pointerEvents = running ? 'none' : 'auto';
    }

    // Cinematic mode: keep the live feed running always (continuous video)
    const isCinematic = currentMode === 'cinematic';

    if (isCinematic) {
        // Always show live stream in cinematic — never switch to still frame
        if (els.mjpegFeed) {
            els.mjpegFeed.style.display = 'block';
            // Refresh the MJPEG connection at the start of each move so any
            // prior stale/dropped stream gets a clean reconnect immediately.
            if (running) els.mjpegFeed.src = `/video_feed?t=${Date.now()}`;
        }
        if (els.latestFrame) els.latestFrame.style.display = 'none';
        const feedLabel = document.getElementById('feedLabel');
        if (feedLabel) feedLabel.innerText =
            'Optical Feed — Live (640×360) + Focus Loupe';
    } else {
        // Timelapse / macro: pause feed during sequence, show last frame
        if (els.mjpegFeed) els.mjpegFeed.style.display = running ? 'none' : 'block';
        if (els.latestFrame) els.latestFrame.style.display = running ? 'block' : 'none';
        const feedLabel = document.getElementById('feedLabel');
        if (feedLabel) feedLabel.innerText = running
            ? 'Last Captured Frame (live feed paused during sequence)'
            : 'Optical Feed — Framing (640×360) + Focus Loupe';
    }

    // Only hide loupe during non-cinematic running sequences
    if (!isCinematic) {
        setLoupeVisible(!running);
    }

    // Progress estimates panel
    const pe = document.getElementById('progress_estimates');
    if (pe) pe.style.display = running ? 'block' : 'none';

    if (els.progressMsg) {
        els.progressMsg.innerText = running ? 'Sequence running…' : 'Idle';
    }

    if (running && !isCinematic) {
        if (!latestFrameInterval) {
            latestFrameInterval = setInterval(() => {
                if (els.latestFrame) els.latestFrame.src = `/latest_frame?t=${Date.now()}`;
            }, 2000);
        }
        stopLoupePolling();
    } else {
        if (latestFrameInterval) { clearInterval(latestFrameInterval); latestFrameInterval = null; }
        if (!running || isCinematic) startLoupePolling();
    }

    // Status line
    if (running) {
        if (els.statusLine) {
            els.statusLine.innerText = 'STATUS: SEQUENCE RUNNING';
            els.statusLine.style.color = 'var(--accent-gold)';
        }
        updateHGExposureLock();
    } else {
        updateDiskSpace();
        if (els.statusLine) {
            els.statusLine.innerText = 'STATUS: IDLE';
            els.statusLine.style.color = 'var(--accent-green)';
        }
        if (els.mjpegFeed) els.mjpegFeed.src = `/video_feed?t=${Date.now()}`;
        updateHGExposureLock();
    }
}

// ─── STOP / RESET DUAL BUTTON ─────────────────────────────────────────────────
function handleStopReset() {
    if (isRunning) {
        // E-Stop — give immediate feedback, server confirms with run_state:false
        sendCmd('stop');
        log('E-Stop sent — sequence halting…');
        const stopBtn = document.getElementById('stopBtn');
        if (stopBtn) { stopBtn.innerText = 'STOPPING…'; stopBtn.style.opacity = '0.5'; }
    } else {
        // Full server restart
        if (confirm('Restart the server?\n\nThis will:\n• Stop all motors & release relays\n• Fully restart the server process\n• Restore default settings\n• Clear calibration, HG settings and keyframes\n\nThe page will reconnect automatically.')) {
            sendCmd('reset_session');
            log('Server restart requested — reconnecting…');
            // Immediately reset frame counter in UI — server will confirm on reconnect
            const cf = document.getElementById('curFrame');
            const tf = document.getElementById('totalFrames');
            if (cf) cf.innerText = '000';
            if (tf) tf.innerText = '000';
        }
    }
}

// ─── SAFE TO MOVE (E-STOP RECOVERY) ──────────────────────────────────────────
function resumeControl() {
    sendCmd('resume_control');
    log('Restoring movement control…');
    const resumeBtn = document.getElementById('resumeBtn');
    if (resumeBtn) { resumeBtn.style.display = 'none'; }
}

function _handleEstopFired() {
    // Show the "Safe to Move" button so user can explicitly re-enable control
    const resumeBtn = document.getElementById('resumeBtn');
    const stopBtn   = document.getElementById('stopBtn');
    if (resumeBtn) { resumeBtn.style.display = 'inline-block'; }
    if (stopBtn)   { stopBtn.innerText = 'E-Stop'; stopBtn.style.opacity = '1'; }
    log('⚠ E-Stop fired — click "✅ Safe to Move" when ready to resume control.');
}

function _handleControlResumed() {
    // Hide the "Safe to Move" button — control is restored
    const resumeBtn = document.getElementById('resumeBtn');
    if (resumeBtn) { resumeBtn.style.display = 'none'; }
    // Restore stop button opacity in case it was dimmed during stopping
    const stopBtn = document.getElementById('stopBtn');
    if (stopBtn) { stopBtn.style.opacity = '1'; }
    log('✅ Movement control restored — motors ready.');
}

// ─── HG EXPOSURE LOCK ─────────────────────────────────────────────────────────
function updateHGExposureLock() {
    const hgEnabled = document.getElementById('hg_enabled')?.checked ?? false;
    const notice = document.getElementById('hg_override_notice');
    const controls = document.getElementById('remote_controls');

    if (hgEnabled) {
        if (notice) notice.style.display = 'block';
        if (controls) { controls.style.opacity = '0.35'; controls.style.pointerEvents = 'none'; }
    } else {
        if (notice) notice.style.display = 'none';
        if (controls) { controls.style.opacity = '1'; controls.style.pointerEvents = 'auto'; }
    }
}

// ─── TRIGGER MODE CHANGE ─────────────────────────────────────────────────────
function onTriggerModeChange(mode, sendToServer = true) {
    const motionSettings = document.getElementById('motion_settings');
    const auxFireBtn = document.getElementById('auxFireBtn');
    const isMotion = mode.startsWith('picam_motion');
    const isAux = mode.startsWith('aux');

    if (motionSettings) motionSettings.style.display = isMotion ? 'block' : 'none';
    if (auxFireBtn) auxFireBtn.style.display = isAux ? 'block' : 'none';

    // Show/hide the canvas ROI box
    setRoiVisible(isMotion);

    // If motion mode active and camera is not picam, show a note
    const noteEl = document.getElementById('motion_camera_note');
    if (noteEl) {
        const cam = document.getElementById('camera_select')?.value;
        noteEl.style.display = (isMotion && cam !== 'picam') ? 'block' : 'none';
    }

    if (sendToServer) sendCmd('set_trigger_mode', { mode });
}

// ─── MOTION DETECTION SETTINGS ───────────────────────────────────────────────
function sendMotionSettings() {
    const roi = overlay.roi;
    sendCmd('set_motion_roi', {
        roi: [roi.x1, roi.y1, roi.x2, roi.y2],
        threshold: parseInt(document.getElementById('motion_threshold')?.value ?? 5000),
        warmup: parseInt(document.getElementById('motion_warmup')?.value ?? 10),
        cooldown: parseFloat(document.getElementById('motion_cooldown')?.value ?? 2.0)
    });
}

// ─── PROGRESS ESTIMATES ───────────────────────────────────────────────────────
function updateProgressEstimates(data) {
    const cur = parseInt(els.curFrame.innerText) || 0;
    const tot = parseInt(els.totFrame.innerText) || 1;
    const pct = tot > 0 ? Math.round(cur / tot * 100) : 0;

    if (els.progressMsg) {
        els.progressMsg.innerText = isRunning
            ? `${pct}% — ${cur} of ${tot} frames`
            : 'Idle';
    }

    // Live interval + estimated end
    if (data.current_interval !== undefined) {
        const ci = document.getElementById('cur_interval');
        if (ci) ci.innerText = data.current_interval;

        const hn = document.getElementById('hg_interval_note');
        if (hn) {
            const hgOn = document.getElementById('hg_enabled')?.checked;
            hn.style.display = hgOn ? 'inline' : 'none';
        }
    }
    if (data.estimated_end) {
        const ee = document.getElementById('est_end');
        if (ee) ee.innerText = data.estimated_end;
    }
    if (data.secs_remaining !== undefined) {
        const er = document.getElementById('est_remaining');
        if (er) {
            const m = Math.floor(data.secs_remaining / 60);
            const s = data.secs_remaining % 60;
            er.innerText = `${m}m ${s}s`;
        }
    }
}

// ─── HG FIELD RESTORE (from init packet) ─────────────────────────────────────
function restoreHGFields(hg) {
    const set = (id, val) => { const el = document.getElementById(id); if (el && val !== undefined) el.value = val; };
    const check = (id, val) => { const el = document.getElementById(id); if (el && val !== undefined) el.checked = val; };

    check('hg_enabled', hg.enabled);
    set('hg_lat', hg.lat);
    set('hg_lon', hg.lon);
    set('hg_tz', hg.tz);
    set('hg_cam_az', hg.cam_az);
    set('hg_cam_alt', hg.cam_alt);
    set('hg_hfov', hg.hfov);
    set('hg_vfov', hg.vfov);
    set('hg_ev_day', hg.ev_day);
    set('hg_ev_golden', hg.ev_golden);
    set('hg_ev_twilight', hg.ev_twilight);
    set('hg_ev_night', hg.ev_night);
    set('hg_k_day', hg.kelvin_day);
    set('hg_k_golden', hg.kelvin_golden);
    set('hg_k_twilight', hg.kelvin_twilight);
    set('hg_k_night', hg.kelvin_night);
    set('hg_int_day', hg.interval_day);
    set('hg_int_golden', hg.interval_golden);
    set('hg_int_twilight', hg.interval_twilight);
    set('hg_int_night', hg.interval_night);
    set('hg_iso_min', hg.iso_min);
    set('hg_iso_max', hg.iso_max);
    set('hg_ap_day', hg.aperture_day);
    set('hg_ap_night', hg.aperture_night);
    set('hg_shutter_max_twilight', hg.shutter_max_twilight);
    set('hg_shutter_max_night',    hg.shutter_max_night);

    updateHGExposureLock();
    updateEvNightHint();
}

// ─── SEQUENCE START ──────────────────────────────────────────────────────────
function startRun() {
    if (isRunning) return;

    const config = getSeqConfig();

    // If WebSocket isn't open yet, wait up to 5 s for it to connect then fire.
    if (!socket || socket.readyState !== WebSocket.OPEN) {
        log("⏳ Waiting for WebSocket link before starting…");
        const deadline = Date.now() + 5000;
        const poll = setInterval(() => {
            if (socket && socket.readyState === WebSocket.OPEN) {
                clearInterval(poll);
                _doStartRun(config);
            } else if (Date.now() > deadline) {
                clearInterval(poll);
                log("⚠ Start failed — server link could not be established. Check Pi connection.");
            }
        }, 100);
        return;
    }

    _doStartRun(config);
}

function _doStartRun(config) {
    if (isRunning) return;   // guard against double-fire if somehow called twice
    log(`Start: ${config.mode.toUpperCase()} — ${config.total_frames} frames @ ${config.interval}s [${config.trigger_mode}]`);
    // Push latest HG settings to backend before starting — no manual Apply button needed
    if (document.getElementById('hg_enabled')?.checked) {
        sendHGSettings();
    }
    // Optimistically update UI so the button doesn't feel dead.
    // If a scheduled start is configured, show the waiting state instead of
    // "SEQUENCE RUNNING" — the server will send scheduled_wait messages.
    if (config.schedule_start) {
        // Lock the button but don't call setRunState(true) — server countdown
        // messages will call setScheduledWaitState() once the worker starts.
        const startBtn = document.getElementById('startBtn');
        if (startBtn) {
            startBtn.innerText = 'WAITING…';
            startBtn.style.opacity = '0.4';
            startBtn.style.pointerEvents = 'none';
        }
        const statusEl = document.getElementById('statusLine');
        if (statusEl) {
            statusEl.innerText = '⏳ SCHEDULED — waiting for start time…';
            statusEl.style.color = 'var(--accent-gold)';
        }
    } else {
        setRunState(true);
    }
    sendCmd('start_run', config);
}

// ─── RECORD + RUN ────────────────────────────────────────────────────────────
function recordAndRun() {
    const config = getSeqConfig();
    const cinePreroll = parseFloat(document.getElementById('cine_preroll')?.value) || 3;
    const preroll = (currentMode === 'cinematic') ? cinePreroll : (config.tl_preroll_s || 0);
    log(`⏺ Record + Run — preroll ${preroll}s, mode: ${currentMode.toUpperCase()}`);
    sendCmd('record_and_run', { ...config, preroll_s: preroll });
    if (currentMode !== 'cinematic') setRunState(true);
}

// ─── SOFT LIMITS ─────────────────────────────────────────────────────────────
function captureLimitNow(axis, which) {
    // Tell backend to record current motor position as limit
    sendCmd('set_limits', { axis, which, value: null });
    log(`Limit captured: ${axis} ${which} = current position`);
}

function setLimitFromField(axis, which) {
    const id = `${axis}_${which}_val`;
    const val = parseFloat(document.getElementById(id)?.value);
    if (isNaN(val)) return;
    sendCmd('set_limits', { axis, which, value: val });
    log(`Limit set: ${axis} ${which} = ${val}°`);
}

function updateLimitsReadout(data) {
    const el = document.getElementById('limits_readout');
    if (!el) return;
    const pMin = data.pan_min ?? '—';
    const pMax = data.pan_max ?? '—';
    const tMin = data.tilt_min ?? '—';
    const tMax = data.tilt_max ?? '—';
    el.innerText = `Pan: ${pMin}° → ${pMax}°   |   Tilt: ${tMin}° → ${tMax}°`;
    // Sync fields
    if (data.pan_min !== undefined) { const f = document.getElementById('pan_min_val'); if (f) f.value = data.pan_min; }
    if (data.pan_max !== undefined) { const f = document.getElementById('pan_max_val'); if (f) f.value = data.pan_max; }
    if (data.tilt_min !== undefined) { const f = document.getElementById('tilt_min_val'); if (f) f.value = data.tilt_min; }
    if (data.tilt_max !== undefined) { const f = document.getElementById('tilt_max_val'); if (f) f.value = data.tilt_max; }
}


// ─── DISK INFO + ALERTS ──────────────────────────────────────────────────────
async function refreshDiskInfo() {
    try {
        const resp = await fetch('/disk_info');
        const data = await resp.json();
        if (data.error) return;

        const freeGB = data.free / 1073741824;
        const totalGB = data.total / 1073741824;
        const pct = data.free / data.total;   // fraction free

        // Estimated frames remaining
        const cam = document.getElementById('camera_select')?.value || 'picam';
        // sony WiFi: only thumbs+XMP saved on Pi (~0.5 MB); sony_usb: full ARW (~30 MB)
        const mbPerFrame = cam === 'sony_usb' ? 30 : cam === 'picam' ? 25 : 0.5;
        const framesLeft = cam === 'sony'
            ? 99999   // files go to camera card, Pi space is not the constraint
            : Math.floor(data.free / (mbPerFrame * 1048576));

        const label = document.getElementById('disk_free_label');
        if (label) {
            label.className = pct < 0.05 ? 'disk-crit' : pct < 0.15 ? 'disk-warn' : 'disk-ok';
            label.innerHTML = `${freeGB.toFixed(1)} GB free &nbsp;(~<b>${framesLeft.toLocaleString()}</b> frames)`;
        }

        // Pre-flight inline warning below frame count
        const total = parseInt(document.getElementById('total_frames')?.value) || 0;
        const warnEl = document.getElementById('disk_preflight_warn');
        if (warnEl) {
            // Sony WiFi: files go to camera card, Pi disk isn't the constraint
            const show = cam !== 'sony' && total > 0 && framesLeft < total;
            warnEl.style.display = show ? 'block' : 'none';
            if (show) warnEl.textContent =
                `⚠ Only ~${framesLeft} frames fit — ${total}-frame sequence may fail mid-run.`;
        }
    } catch (_) { }
}

// Auto-refresh disk info every 15 s
setInterval(refreshDiskInfo, 15000);

function showDiskFullAlert(msg) {
    document.getElementById('diskAlertBanner')?.remove();
    document.getElementById('diskWarnBanner')?.remove();
    const banner = document.createElement('div');
    banner.id = 'diskAlertBanner';
    banner.className = 'disk-alert-banner';
    banner.innerHTML = `<strong>⛔ DISK FULL — SEQUENCE HALTED</strong><br>
        <span style="font-size:0.8rem">${msg}</span>
        <button onclick="this.parentElement.remove()">Dismiss</button>`;
    document.body.appendChild(banner);
    log(msg);
    setRunState(false);
    refreshDiskInfo();
}

function showSeqHaltedAlert(msg) {
    document.getElementById('seqHaltedBanner')?.remove();
    const banner = document.createElement('div');
    banner.id = 'seqHaltedBanner';
    banner.className = 'disk-alert-banner';
    banner.innerHTML = `<strong>⛔ SEQUENCE HALTED</strong><br>
        <span style="font-size:0.8rem">${msg}</span>
        <button onclick="this.parentElement.remove()">Dismiss</button>`;
    document.body.appendChild(banner);
    log(msg);
    setRunState(false);
    refreshDiskInfo();
}

function showDiskWarnAlert(msg) {
    if (document.getElementById('diskAlertBanner')) return;
    document.getElementById('diskWarnBanner')?.remove();
    const banner = document.createElement('div');
    banner.id = 'diskWarnBanner';
    banner.className = 'disk-alert-banner';
    banner.style.cssText = banner.style.cssText +
        ';border-color:var(--accent-gold);color:var(--accent-gold);background:#1a1200;';
    banner.innerHTML = `<strong>⚠ LOW DISK SPACE</strong><br>
        <span style="font-size:0.8rem">${msg}</span>
        <button onclick="this.parentElement.remove()" style="border-color:var(--accent-gold)">Dismiss</button>`;
    document.body.appendChild(banner);
    log(msg);
    setTimeout(() => document.getElementById('diskWarnBanner')?.remove(), 30000);
}

// ─── PATH LIMIT WARNING MODAL ────────────────────────────────────────────────
function showPathLimitWarning(data) {
    document.getElementById('pathLimitModal')?.remove();
    const { context, violations } = data;
    const action = context === 'cinematic' ? 'Play Move' : 'Start Sequence';

    const violationHtml = violations.map(v => {
        const dir = v.end === 'min' ? '←' : '→';
        return `
        <div style="display:flex; justify-content:space-between; align-items:center;
                    padding:5px 0; border-bottom:1px solid #2a2a2a; font-size:0.78rem; gap:8px;">
            <span style="color:var(--accent-gold); white-space:nowrap; font-weight:600;">
                ${dir} ${v.axis} ${v.end.toUpperCase()}
            </span>
            <span style="color:var(--text-muted);">
                needs&nbsp;<b style="color:#ccc">${v.needed}${v.unit}</b>
                &nbsp;·&nbsp;limit&nbsp;${v.current}${v.unit}
            </span>
            <span style="color:#c55; white-space:nowrap;">+${v.expand_by}${v.unit} over</span>
        </div>`;
    }).join('');

    const modal = document.createElement('div');
    modal.id = 'pathLimitModal';
    modal.style.cssText = `
        position:fixed; inset:0; z-index:9000;
        background:rgba(0,0,0,0.78); backdrop-filter:blur(4px);
        display:flex; align-items:center; justify-content:center;`;
    modal.innerHTML = `
        <div style="background:#181818; border:1px solid var(--accent-gold);
                    border-radius:12px; padding:22px 24px; max-width:480px; width:92%;
                    box-shadow:0 8px 48px rgba(0,0,0,0.7);">
            <div style="font-size:1.05rem; font-weight:700; color:var(--accent-gold);
                        margin-bottom:10px;">⚠ Motion Path Outside Soft Limits</div>
            <div style="font-size:0.78rem; color:var(--text-dim); margin-bottom:14px; line-height:1.5;">
                The curved path between keyframes swings outside the calibrated safe zone.
                Without expansion the rig will stall at the boundary mid-move.
            </div>
            <div style="border:1px solid #2a2a2a; border-radius:6px;
                        padding:8px 12px; margin-bottom:14px;">
                ${violationHtml}
            </div>
            <div style="font-size:0.72rem; color:var(--text-muted); margin-bottom:20px;
                        background:#111; border-radius:6px; padding:8px 10px; line-height:1.5;">
                <b style="color:#aaa;">Expand Limits</b> widens the soft limits to cover the full
                path with a small safety margin. Confirm the expanded range is physically safe
                before running.
            </div>
            <div style="display:flex; gap:10px;">
                <button onclick="confirmExpandLimits()"
                        style="flex:2; padding:10px 14px; background:var(--accent-gold);
                               color:#000; font-weight:700; border:none; border-radius:8px;
                               cursor:pointer; font-size:0.85rem;">
                    Expand Limits &amp; ${action}
                </button>
                <button onclick="cancelPathPlay()"
                        style="flex:1; padding:10px; background:none;
                               border:1px solid #444; color:var(--text-muted);
                               border-radius:8px; cursor:pointer; font-size:0.85rem;">
                    Cancel
                </button>
            </div>
        </div>`;
    document.body.appendChild(modal);
    log(`⚠ Path exceeds soft limits on ${violations.length} bound${violations.length > 1 ? 's' : ''} — confirm or cancel.`);
}

function confirmExpandLimits() {
    document.getElementById('pathLimitModal')?.remove();
    sendCmd('path_expand_and_play');
}

function cancelPathPlay() {
    document.getElementById('pathLimitModal')?.remove();
    sendCmd('path_cancel_play');
}

// ─── HIGH-RES LOUPE POLLING ───────────────────────────────────────────────────
// Polls /loupe_crop at 2fps. The returned JPEG is used by drawOverlay()
// instead of sampling from the low-res MJPEG stream.
const loupeCropImage = new Image();  // shared Image object reused each poll
let _loupePollTimer = null;

function startLoupePolling() {
    if (_loupePollTimer) return;
    _loupePollTimer = setInterval(async () => {
        if (!overlay?.loupe?.visible) return;
        const l = overlay.loupe;
        const cw = overlay.cw || overlay.canvas?.clientWidth || 640;
        const ch = overlay.ch || overlay.canvas?.clientHeight || 480;

        const cx = l.x.toFixed(3);
        const cy = l.y.toFixed(3);

        // Container is exactly 4:3 — image fills full width with no letterbox bars.
        // Crop radius in frame-fraction = loupe_radius_px / canvas_width / zoom
        // e.g. r=180px, cw=960px, zoom=4 → rFrac = 0.047 → tight 4× zoom crop
        const rFrac = (l.r / cw / l.zoom).toFixed(4);

        try {
            const url = `/loupe_crop?cx=${cx}&cy=${cy}&r=${rFrac}&t=${Date.now()}`;
            const resp = await fetch(url);
            if (!resp.ok) return;
            const blob = await resp.blob();
            const objUrl = URL.createObjectURL(blob);
            loupeCropImage.onload = () => URL.revokeObjectURL(loupeCropImage._prevUrl);
            loupeCropImage._prevUrl = objUrl;
            loupeCropImage.src = objUrl;
        } catch (_) { }
    }, 500);   // 2fps — enough for focus checking
}

function stopLoupePolling() {
    clearInterval(_loupePollTimer);
    _loupePollTimer = null;
}

// ─── CAMERA SWITCHING ────────────────────────────────────────────────────────
function onCameraChange(val, isInit = false) {
    els.sonyGuide.style.display = (val === 'sony') ? 'block' : 'none';
    const usbGuide = document.getElementById('sony_usb_guide');
    if (usbGuide) usbGuide.style.display = (val === 'sony_usb') ? 'block' : 'none';
    const s2Guide = document.getElementById('sony_s2_guide');
    if (s2Guide) s2Guide.style.display = (val === 'sony_s2') ? 'block' : 'none';

    // HG is only compatible with picam, sony-wifi, and sony-usb
    // S2 cable modes have no API link — disable HG checkbox
    const hgCb = document.getElementById('hg_enabled');
    const hgRow = hgCb?.closest('.control-item') || hgCb?.parentElement;
    const hgIncompat = (val === 'sony_s2' || val === 's2');
    if (hgCb) {
        hgCb.disabled = hgIncompat;
        if (hgIncompat && hgCb.checked) {
            hgCb.checked = false;
            updateHGExposureLock();
        }
    }
    if (hgRow) hgRow.style.opacity = hgIncompat ? '0.4' : '';

    updatePreviewToggleVisibility();
    // Cache-bust the MJPEG stream
    els.mjpegFeed.src = `/video_feed?t=${Date.now()}`;

    // Skip connection actions and set_camera echo when restoring from init packet —
    // the server already knows the camera state and the WiFi handshake is user-initiated.
    if (isInit) return;

    if (val === 'sony') {
        log("wlan1: Triggering headless handshake…");
        sendCmd('connect_camera_wifi');
    } else if (val === 'sony_usb') {
        log("Sony USB mode: tethered via gphoto2. Holy Grail available. Detecting camera…");
        detectSonyUsb();
        // Reset liveview button state when entering USB mode
        _usbLiveviewOn = false;
        const btn = document.getElementById('usbLiveviewBtn');
        if (btn) { btn.textContent = '📹 Liveview OFF'; btn.style.background = ''; }
    } else if (val === 'sony_s2') {
        log("Sony S2 mode: hardware shutter cable active. ~5ms trigger latency. Holy Grail unavailable.");
    } else if (val === 's2') {
        log("S2 mode: manual control on camera body.");
    } else {
        log("PiCam mode: full remote control active.");
    }
    sendCmd('set_camera', val);
}

function detectSonyUsb() {
    const statusEl = document.getElementById('sony_usb_status');
    if (statusEl) statusEl.textContent = '🔍 Scanning USB for Sony camera…';
    sendCmd('detect_sony_usb');
}

let _usbLiveviewOn = false;
function toggleUsbLiveview() {
    _usbLiveviewOn = !_usbLiveviewOn;
    const btn = document.getElementById('usbLiveviewBtn');
    if (_usbLiveviewOn) {
        sendCmd('sony_usb_liveview_start');
        // Switch preview to sony_usb feed and bust cache
        els.mjpegFeed.src = `/video_feed?t=${Date.now()}`;
        if (btn) { btn.textContent = '📹 Liveview ON'; btn.style.background = 'var(--accent-green,#3a7)'; }
        log('Sony USB liveview started (~2-3fps). Frame updates will appear in preview.');
    } else {
        sendCmd('sony_usb_liveview_stop');
        if (btn) { btn.textContent = '📹 Liveview OFF'; btn.style.background = ''; }
        log('Sony USB liveview stopped.');
    }
}

// ─── WEBSOCKET ENGINE ────────────────────────────────────────────────────────
// Ping interval — keeps the TCP connection alive through NAT/router idle timeouts.
// Most home routers drop idle connections after 60–120 s; 25 s ping prevents this.
const WS_PING_INTERVAL_MS = 25000;
let _wsPingTimer = null;

function _startWsPing() {
    _stopWsPing();
    _wsPingTimer = setInterval(() => {
        if (socket && socket.readyState === WebSocket.OPEN) {
            sendCmd('ping');
        }
    }, WS_PING_INTERVAL_MS);
}

function _stopWsPing() {
    if (_wsPingTimer) { clearInterval(_wsPingTimer); _wsPingTimer = null; }
}

function connectWS() {
    // Close any existing broken socket cleanly before creating a new one
    if (socket && socket.readyState !== WebSocket.CLOSED) {
        socket.onclose = null;   // prevent recursive reconnect from old socket
        socket.close();
    }
    _stopWsPing();

    socket = new WebSocket(WS_URL);

    socket.onopen = () => {
        log("WebSocket: High-speed link ACTIVE.");
        els.statusLine.innerText = "STATUS: CONNECTED";
        els.statusLine.style.color = "var(--accent-green)";
        _stopAllUiNudges();   // clear any stale timers from before reconnect
        _startWsPing();   // keep NAT alive
    };

    socket.onmessage = (event) => {
        try {
            handleIncomingData(JSON.parse(event.data));
        } catch (e) {
            console.warn("WS parse error:", e);
        }
    };

    socket.onclose = () => {
        _stopWsPing();
        _stopAllUiNudges();   // kill keepalive timers so they don't restart motors on reconnect
        // Clear any active gamepad mirror state so stale highlights don't linger
        _clearGamepadMirror();
        els.statusLine.innerText = "STATUS: LINK SEVERED — RECONNECTING…";
        els.statusLine.style.color = "var(--stop-red)";
        if (isRunning) {
            // Do NOT call setRunState(false) — the server sequence keeps running.
            // The init packet on reconnect will restore the correct state.
            log("⚠ Connection lost. Server sequence likely still running. Reconnecting…");
        }
        setTimeout(connectWS, RECONNECT_DELAY);
    };

    socket.onerror = (err) => {
        console.error("WebSocket error:", err);
        // onerror always fires before onclose — onclose will handle reconnect
    };
}

function sendCmd(command, value = null) {
    if (socket && socket.readyState === WebSocket.OPEN) {
        const payload = (value !== null && typeof value === 'object')
            ? { command, ...value }
            : { command, value };
        socket.send(JSON.stringify(payload));
    }
}

// ─── INCOMING DATA ROUTER ────────────────────────────────────────────────────
function handleIncomingData(data) {

    // ── Kicked by newer client ────────────────────────────────────────────────
    if (data.type === "kicked") {
        log("⚠ " + data.msg);
        // Show a prominent overlay so the user knows this tab is dead
        _showKickedBanner(data.msg);
        // Stop all polling — this tab is no longer in control
        stopLoupePolling();
        if (latestFrameInterval) { clearInterval(latestFrameInterval); latestFrameInterval = null; }
        // Prevent the onclose reconnect loop from spinning up again
        if (socket) { socket.onclose = null; socket.onerror = null; }
        return;
    }

    // ── Full state restore on connect / reset ─────────────────────────────────
    if (data.type === "init") {
        if (data.scheduled_waiting) {
            // Server is in the scheduled-wait countdown — don't show "RUNNING".
            // The next scheduled_wait message from the server will populate the
            // countdown; for now just lock the Start button with a placeholder.
            setScheduledWaitState(0, data.scheduled_target || '');
        } else {
            setRunState(data.running || false);
        }

        // Frame counters
        if (data.current_frame !== undefined) els.curFrame.innerText = String(data.current_frame).padStart(3, '0');
        if (data.total_frames !== undefined) {
            els.totFrame.innerText = data.total_frames;
            const tf = document.getElementById('total_frames');
            if (tf) tf.value = data.total_frames;
        }

        // Axis positions
        if (data.pan_deg !== undefined) { els.valP.innerText = data.pan_deg.toFixed(1); calibState.pan_deg = data.pan_deg; }
        if (data.tilt_deg !== undefined) { els.valT.innerText = data.tilt_deg.toFixed(1); calibState.tilt_deg = data.tilt_deg; _updatePerspCurrentTilt(data.tilt_deg); }
        if (data.slider_mm !== undefined) els.valS.innerText = data.slider_mm.toFixed(1);

        // Save path
        if (data.save_path) {
            const sp = document.getElementById('save_path');
            if (sp) sp.value = data.save_path;
        }

        // Trigger mode
        if (data.trigger_mode) {
            const r = document.querySelector(`input[name="trigger_mode"][value="${data.trigger_mode}"]`);
            if (r) { r.checked = true; onTriggerModeChange(data.trigger_mode, false); }
        }

        // Motion ROI
        if (data.motion_roi) {
            setRoiFromData(data.motion_roi);
        }
        if (data.motion_threshold !== undefined) {
            const el = document.getElementById('motion_threshold');
            if (el) el.value = data.motion_threshold;
        }
        if (data.motion_warmup !== undefined) {
            const el = document.getElementById('motion_warmup');
            if (el) el.value = data.motion_warmup;
        }
        if (data.motion_cooldown !== undefined) {
            const el = document.getElementById('motion_cooldown');
            if (el) el.value = data.motion_cooldown;
        }

        // HG settings
        if (data.hg_settings) restoreHGFields(data.hg_settings);

        // Active camera — pass isInit=true to restore UI only, no WiFi handshake
        if (data.active_camera) {
            const cs = document.getElementById('camera_select');
            if (cs) { cs.value = data.active_camera; onCameraChange(data.active_camera, true); }
        }

        // Camera orientation
        if (data.camera_orientation) {
            _applyOrientationUI(data.camera_orientation);
        }

        // Cinematic fps
        if (data.cine_fps) {
            _applyCineFpsUI(data.cine_fps);
        }

        // Motor Inversions
        if (data.slider_inverted !== undefined) {
            const el = document.getElementById('side_slider_inv');
            if (el) el.checked = data.slider_inverted;
        }
        if (data.pan_inverted !== undefined) {
            const el = document.getElementById('side_pan_inv');
            if (el) el.checked = data.pan_inverted;
        }
        if (data.tilt_inverted !== undefined) {
            const el = document.getElementById('side_tilt_inv');
            if (el) el.checked = data.tilt_inverted;
        }

        // Vibe / exp margin / interval / sidecar settings
        if (data.vibe_delay   !== undefined) { const el = document.getElementById('vibe_delay');   if (el) el.value = data.vibe_delay; }
        if (data.exp_margin   !== undefined) { const el = document.getElementById('exp_margin');   if (el) el.value = data.exp_margin; }
        if (data.tl_preroll_s !== undefined) { const el = document.getElementById('tl_preroll'); if (el) el.value = data.tl_preroll_s; }
        if (data.manual_interval !== undefined) {
            const el = document.getElementById('manual_interval');
            if (el) {
                el.value = data.manual_interval;
                log(`📋 Session restored interval: ${data.manual_interval}s — change this field if needed before starting.`);
            }
        }

        // Limits
        updateLimitsReadout(data);
        updateCalibReadout();

        // Restore active mode UI without sending set_mode to the backend —
        // the server already knows its mode; we're just catching the UI up.
        if (data.active_mode) {
            _applyModeUI(data.active_mode);
        }

        // Gamepad indicator — restore connected state if controller was already
        // paired before this browser tab opened (the event-based gamepad_status
        // message is only broadcast at connect/disconnect time, so a fresh page
        // load misses it if the controller is already running).
        if (data.gamepad_connected !== undefined) {
            handleGamepadStatus({ connected: data.gamepad_connected });
        }

        if (data.running) {
            const modeLabel = (data.active_mode || 'timelapse').toUpperCase();
            log(`Reconnected — ${modeLabel} sequence IN PROGRESS (frame ${data.current_frame} of ${data.total_frames})`);
            // Status line stays green/running — setRunState above handles it
        } else if (data.interrupted) {
            // Server was restarted mid-sequence (process kill, power blip, etc.)
            // Show in log only — don't overwrite status line with alarming text
            log(`⚠ Server was restarted. Previous sequence stopped at frame ${data.current_frame}. Settings restored. Ready to start a new run.`);
            const statusEl = document.getElementById('statusLine');
            if (statusEl) {
                statusEl.innerText = `STATUS: READY (restarted at frame ${data.current_frame})`;
                statusEl.style.color = 'var(--accent-gold)';
            }
        } else {
            log('Connected — system idle.');
        }

        // Show the last stop reason prominently if available (why the sequence ended)
        if (data.stop_reason) {
            log(`Last session: ${data.stop_reason}`);
            if (data.crash_report_path) {
                log(`📄 Crash report saved to: ${data.crash_report_path}`);
            }
            const statusEl = document.getElementById('statusLine');
            if (statusEl && !data.running && !data.interrupted) {
                const isError = data.stop_reason.startsWith('⛔');
                statusEl.innerText = `STATUS: ${isError ? 'HALTED — SEE LOG' : 'IDLE — last sequence ended'}`;
                statusEl.style.color = isError ? 'var(--stop-red)' : 'var(--accent-gold)';
            }
        }
    }

    // ── Stop reason broadcast ─────────────────────────────────────────────────
    if (data.type === "stop_reason") {
        log(`${data.msg}`);
    }

    // ── Run state change ──────────────────────────────────────────────────────
    if (data.type === "run_state") {
        setRunState(data.running);
        if (!data.running) _clearReturnToStartBusy();
    }

    // ── E-stop recovery ───────────────────────────────────────────────────────
    if (data.type === "estop_fired")     { _handleEstopFired(); }
    if (data.type === "control_resumed") { _handleControlResumed(); }

    // ── Scheduled wait countdown ──────────────────────────────────────────────
    if (data.type === "scheduled_wait") {
        setScheduledWaitState(data.seconds_remaining, data.target_time);
    }
    if (data.type === "scheduled_wait_done" || data.type === "scheduled_wait_cancelled") {
        clearScheduledWaitState();
    }

    // ── Status / telemetry ────────────────────────────────────────────────────
    if (data.type === "status") {
        if (data.nodes !== undefined)
            els.nodeReadout?.innerText && (els.nodeReadout.innerText = `Nodes: ${data.nodes}`);

        if (data.frame !== undefined)
            els.curFrame.innerText = String(data.frame).padStart(3, '0');
        if (data.total !== undefined)
            els.totFrame.innerText = data.total;

        if (data.pos_s !== undefined) els.valS.innerText = data.pos_s.toFixed(1);
        if (data.pos_p !== undefined) {
            els.valP.innerText = data.pos_p.toFixed(1);
            calibState.pan_deg = data.pos_p;
            updateCalibReadout();
        }
        if (data.pos_t !== undefined) {
            els.valT.innerText = data.pos_t.toFixed(1);
            calibState.tilt_deg = data.pos_t;
            _updatePerspCurrentTilt(data.pos_t);
        }

        // Update cinematic limit position bars with live position
        if (data.pos_s !== undefined && data.pos_p !== undefined && data.pos_t !== undefined) {
            _updateLimitBars(data.pos_s, data.pos_p, data.pos_t);
        }

        // HG telemetry
        if (data.hg_phase !== undefined) els.hgPhase.innerText = data.hg_phase;
        if (data.hg_sun_alt !== undefined) els.hgSunAlt.innerText = parseFloat(data.hg_sun_alt).toFixed(1);
        if (data.hg_ev !== undefined) els.hgEV.innerText = parseFloat(data.hg_ev).toFixed(2);
        if (data.hg_iso !== undefined) els.hgISO.innerText = data.hg_iso;
        if (data.hg_shutter !== undefined) els.hgShutter.innerText = data.hg_shutter;
        if (data.hg_kelvin !== undefined) els.hgKelvin.innerText = data.hg_kelvin;

        // Sequence progress estimates
        updateProgressEstimates(data);
    }

    // ── Shutter flash + latest frame ─────────────────────────────────────────
    if (data.type === "shutter_event") {
        if (els.shutterIndicator) {
            els.shutterIndicator.innerText = "FIRING…";
            els.shutterIndicator.style.color = "var(--accent-red)";
        }
        if (isRunning && els.latestFrame)
            els.latestFrame.src = `/latest_frame?t=${Date.now()}`;
        setTimeout(() => {
            if (els.shutterIndicator) {
                els.shutterIndicator.innerText = "READY";
                els.shutterIndicator.style.color = "var(--text-dim)";
            }
        }, 600);
    }

    if (data.type === "sony_status") updateSonyStatus(data);
    if (data.type === "sony_scan_status") {
        const statusEl = document.getElementById('sony_status');
        if (statusEl) { statusEl.innerText = data.msg; statusEl.style.color = 'var(--accent-gold)'; }
    }
    if (data.type === "sony_scan_result") handleSonyScanResult(data);
    if (data.type === "limits_updated") updateLimitsReadout(data);
    if (data.type === "inversions_updated") handleInversionsUpdated(data);
    if (data.type === "focal_detected") {
        log(data.msg);
        _showFocalDetectedBanner(data.focal_mm, data.lens_model);
    }
    if (data.type === "lens_info") {
        _applyLensInfo(data);
    }
    if (data.type === "log") log(data.msg);
    if (data.type === "disk_full")   showDiskFullAlert(data.msg);
    if (data.type === "seq_halted")  showSeqHaltedAlert(data.msg);
    if (data.type === "disk_warn")   showDiskWarnAlert(data.msg);
    if (data.type === "path_limit_warning") showPathLimitWarning(data);
    if (data.type === "folder_created") { browseTo(data.path); log(`Folder created: ${data.path}`); }
    if (data.type === "preview_camera_changed") {
        const btn = document.getElementById('previewToggleBtn');
        if (btn) btn.textContent = data.camera === 'sony' ? '🔁 Sony' : '📷 PiCam';
    }

    if (data.type === "disk_full") {
        handleDiskFull(data);
        setRunState(false);
    }

    if (data.type === "disk_warn") handleDiskWarn(data);
    if (data.type === "sony_storage_warn") handleSonyStorageWarn(data);
    if (data.type === "sony_record_error") handleSonyRecordError(data);
    if (data.type === "sony_usb_status") handleSonyUsbStatus(data);
    if (data.type === "folder_created") handleFolderCreated(data);

    // ── Macro mode messages ───────────────────────────────────────────────────
    if (data.type === "macro_rail_mark") { handleMacroRailMark(data); return; }
    if (data.type === "macro_rotation_mark") { handleMacroRotMark(data); return; }
    if (data.type === "macro_tilt_mark") { handleMacroTiltMark(data); return; }
    if (data.type === "macro_aux_mark") { handleMacroAuxMark(data); return; }
    if (data.type === "macro_progress") { handleMacroProgress(data); return; }
    if (data.type === "macro_stack_complete") { handleMacroStackComplete(data); return; }
    if (data.type === "macro_done") { handleMacroDone(data); return; }
    if (data.type === "macro_lens_profiles") { handleMacroLensProfiles(data); return; }
    if (data.type === "macro_easing_curves") { handleMacroEasingCurves(data); return; }
    if (data.type === "macro_grid_computed") { handleMacroGridComputed(data); return; }

    // ── Hardware reference zero ────────────────────────────────────────────
    if (data.type === "hardware_zeroed") { handleHardwareZeroed(data); return; }

    // ── Cinematic ──────────────────────────────────────────────────────────
    if (data.type === "camera_orientation") { handleCameraOrientation(data); return; }
    if (data.type === "cinematic_limits") { handleCineLimits(data); return; }
    if (data.type === "cinematic_keyframes") { handleCineKeyframes(data); return; }
    if (data.type === "cinematic_keyframe_added") {
        _cineKeyframes.push(data); _renderKeyframeList(); return;
    }
    if (data.type === "cinematic_progress") { handleCineProgress(data); return; }
    if (data.type === "cinematic_play_done") { handleCinePlayDone(); return; }
    if (data.type === "cinematic_origin_set")      { handleCineOriginSet(data); return; }
    if (data.type === "cinematic_reference_saved") { handleCineRefSaved(data); return; }
    if (data.type === "cinematic_reference_cleared") { handleCineRefCleared(); return; }
    if (data.type === "cinematic_moves") { handleCineMoves(data); return; }
    if (data.type === "cinematic_state") { handleCineState(data); return; }
    if (data.type === "cinematic_global_easing") {
        _globalEasing = data.curve;
        _applyGlobalEasing(data.curve);
        _renderKeyframeList();   // refresh override labels
        return;
    }
    if (data.type === "cinematic_inertia") {
        const m = document.getElementById('cine_mass');
        const d = document.getElementById('cine_drag');
        const s = document.getElementById('cine_pt_scale');
        if (m) { m.value = data.mass; updateInertiaLabel('mass'); }
        if (d) { d.value = data.drag; updateInertiaLabel('drag'); }
        if (s && data.pan_tilt_scale !== undefined) {
            s.value = ptScaleToSlider(data.pan_tilt_scale);
            updatePtScaleLabel();
        }
        return;
    }
    if (data.type === "arctan_status") { handleArctanStatus(data); return; }
    if (data.type === "arctan_enabled") { handleArctanEnabled(data); return; }
    if (data.type === "record_state") { handleRecordState(data); return; }
    if (data.type === "gamepad_btn")   { handleGamepadBtn(data);    return; }
    if (data.type === "gamepad_status") { handleGamepadStatus(data); return; }
    if (data.type === "gamepad_input") { handleGamepadInput(data);  return; }
    if (data.type === "pong")          { return; }   // keepalive reply — no action needed
    if (data.type === "cinematic_status") { log(data.msg); return; }

    if (data.type === "preview_camera_changed") {
        const btn = document.getElementById('previewToggleBtn');
        if (btn) btn.innerText = data.camera === 'sony' ? '📷 Sony' : '📷 PiCam';
        if (els.mjpegFeed) els.mjpegFeed.src = `/video_feed?t=${Date.now()}`;
    }

    if (data.type === "calibration_done") {
        calibState.calibrated = true;
        calibState.origin_az = data.origin_az;
        updateCalibReadout();
        const f1 = document.getElementById('hg_cam_az');
        const f2 = document.getElementById('hg_cam_alt');
        if (f1) f1.value = data.cam_az.toFixed(1);
        if (f2) f2.value = data.cam_alt.toFixed(1);
    }
}

// ─── JOYSTICK ENGINE ────────────────────────────────────────────────────────
let sliderStripActive = false;
let sliderVz = 0;

function setupJoystick() {
    // Track WHICH pointer activated the joystick so we don't confuse a second
    // finger (elsewhere on screen) with joystick input.  Multi-touch was the
    // primary source of "runaway" motors: pointerdown on joystick captured
    // pointer A, but pointermove on window fired for pointer B (different
    // finger), computing offsets relative to the joystick centre → large
    // spurious vx/vy → full-speed motor command.
    let _capturedPointerId = null;
    let _lastJoySendMs = 0;          // throttle: max 50 Hz (one send per 20 ms)

    const _releaseJoystick = (e) => {
        // Only process the specific pointer that activated the joystick.
        if (_capturedPointerId !== null && e && e.pointerId !== _capturedPointerId) return;
        joystickActive = false;
        _capturedPointerId = null;
        els.joystickKnob.style.left = '50%';
        els.joystickKnob.style.top = '50%';
        // Zero pan/tilt; preserve slider strip velocity.
        // Server calls instant_stop_pt() on vx=vy=0 — no inertia coast.
        sendCmd('joystick', { vx: 0, vy: 0, vz: sliderVz });
    };

    const handleMove = (e) => {
        // Ignore moves from other pointers or when not active.
        if (!joystickActive || _capturedPointerId === null) return;
        if (e.pointerId !== _capturedPointerId) return;

        const rect = els.joystickPad.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        const maxR = rect.width / 2;
        let dx = e.clientX - cx;
        let dy = e.clientY - cy;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > maxR) { dx *= maxR / dist; dy *= maxR / dist; }
        const vx = dx / maxR;
        const vy = -(dy / maxR);
        // Always update the knob position visually so it tracks the finger smoothly.
        els.joystickKnob.style.left = `calc(50% + ${dx}px)`;
        els.joystickKnob.style.top = `calc(50% + ${dy}px)`;
        // Throttle network sends to 50 Hz (20 ms minimum gap).
        // Browser pointermove fires at display refresh rate (60–120 Hz); sending
        // every frame wastes bandwidth and can cause a backlog that makes the
        // motor appear to lag then lurch when the connection clears.
        // The InertiaEngine runs at 50 Hz so sending faster than that has no effect.
        const _now = Date.now();
        if (_now - _lastJoySendMs < 20) return;
        _lastJoySendMs = _now;
        sendCmd('joystick', { vx, vy, vz: sliderVz });
    };

    els.joystickPad.addEventListener('pointerdown', (e) => {
        // If already tracking a pointer (shouldn't happen, but be safe), release first
        if (_capturedPointerId !== null) {
            try { els.joystickPad.releasePointerCapture(_capturedPointerId); } catch(_) {}
        }
        joystickActive = true;
        _capturedPointerId = e.pointerId;
        els.joystickPad.setPointerCapture(e.pointerId);
        handleMove(e);
    });

    // Attach move/up/cancel to the joystick element itself.
    // With setPointerCapture active, these fire even when the finger drifts
    // outside the element — no need for window-level listeners.
    els.joystickPad.addEventListener('pointermove', handleMove);

    els.joystickPad.addEventListener('pointerup', (e) => {
        try { els.joystickPad.releasePointerCapture(e.pointerId); } catch(_) {}
        _releaseJoystick(e);
    });

    // pointercancel fires when the OS steals the gesture (e.g. scroll, palm
    // rejection, incoming call).  Without this handler the captured pointer is
    // silently discarded and the joystick stays "active" forever → runaway.
    els.joystickPad.addEventListener('pointercancel', (e) => {
        try { els.joystickPad.releasePointerCapture(e.pointerId); } catch(_) {}
        _releaseJoystick(e);
    });

    // Fallback: window-level pointerup catches any edge case where the event
    // doesn't reach the element (rare, but defensive).
    window.addEventListener('pointerup', (e) => {
        if (joystickActive && e.pointerId === _capturedPointerId) {
            _releaseJoystick(e);
        }
    });
}

// ─── SLIDER STRIP — see full implementation below ────────────────────────────

// ─── SONY WIFI CONNECTION ────────────────────────────────────────────────────
// Auto-detect Sony pre-connected to wlan1 (no SSID/password needed).
// Sends check_sony_connection; backend discovers the camera's IP from wlan1 gateway.
function detectSony() {
    const statusEl = document.getElementById('sony_status');
    if (statusEl) {
        statusEl.innerText = "Detecting… checking wlan1";
        statusEl.style.color = "var(--accent-gold)";
    }
    sendCmd('check_sony_connection');
    log("Sony: checking wlan1 for pre-connected camera…");
}

function scanSonyNetworks() {
    const btn = document.getElementById('sonyScanBtn');
    if (btn) { btn.textContent = '⏳ Scanning…'; btn.disabled = true; }
    const statusEl = document.getElementById('sony_status');
    if (statusEl) {
        statusEl.innerText = 'Scanning wlan1 for camera networks…';
        statusEl.style.color = 'var(--accent-gold)';
    }
    // Clear previous results
    const listDiv = document.getElementById('sony_network_list');
    if (listDiv) listDiv.innerHTML = '';
    const resultsDiv = document.getElementById('sony_scan_results');
    if (resultsDiv) resultsDiv.style.display = 'none';
    sendCmd('sony_wifi_scan');
    log('Sony: scanning wlan1 for networks…');
}

function handleSonyScanResult(data) {
    // Re-enable scan button
    const btn = document.getElementById('sonyScanBtn');
    if (btn) { btn.textContent = '🔍 Scan for Camera'; btn.disabled = false; }

    const resultsDiv = document.getElementById('sony_scan_results');
    const listDiv    = document.getElementById('sony_network_list');
    const statusEl   = document.getElementById('sony_status');
    if (!resultsDiv || !listDiv) return;

    listDiv.innerHTML = '';

    if (data.error) {
        if (statusEl) {
            statusEl.innerText = 'Scan failed: ' + data.error;
            statusEl.style.color = 'var(--accent-red)';
        }
        listDiv.innerHTML = `<div style="color:var(--accent-red);font-size:0.72rem;padding:4px;">Scan error: ${data.error}</div>`;
        resultsDiv.style.display = 'block';
        return;
    }

    if (!data.networks || data.networks.length === 0) {
        if (statusEl) {
            statusEl.innerText = 'No networks found on wlan1. Is the camera on?';
            statusEl.style.color = 'var(--accent-red)';
        }
        listDiv.innerHTML = '<div style="color:var(--text-dim);font-size:0.72rem;padding:4px;">No networks found. Turn camera on and try again.</div>';
        resultsDiv.style.display = 'block';
        return;
    }

    if (statusEl) {
        statusEl.innerText = `Found ${data.networks.length} network(s) — tap one to select`;
        statusEl.style.color = 'var(--accent-gold)';
    }

    data.networks.forEach(n => {
        const isSony = n.ssid.startsWith('DIRECT-') ||
                       n.ssid.toUpperCase().includes('SONY') ||
                       n.ssid.toUpperCase().includes('ILCE');
        const item = document.createElement('div');
        item.style.cssText = `
            padding: 5px 8px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 0.75rem;
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: ${isSony ? 'rgba(34,170,100,0.15)' : 'rgba(255,255,255,0.04)'};
            border: 1px solid ${isSony ? 'var(--accent-green,#3a7)' : '#333'};
            color: ${isSony ? 'var(--accent-green,#3a7)' : 'var(--text-dim)'};
        `;
        const sigBar  = Math.round((parseInt(n.signal) || 0) / 20);  // 0-5 bars
        const sigText = '▮'.repeat(sigBar) + '▯'.repeat(5 - sigBar);
        item.innerHTML = `
            <span>${n.ssid}${isSony ? ' 📷' : ''}</span>
            <span style="font-size:0.65rem;opacity:0.7;letter-spacing:-1px;">${sigText} ${n.signal}%</span>
        `;
        item.onclick = () => {
            const ssidInput = document.getElementById('sony_ssid');
            if (ssidInput) ssidInput.value = n.ssid;
            // Highlight selection
            listDiv.querySelectorAll('div').forEach(d => d.style.outline = '');
            item.style.outline = '2px solid var(--accent-teal,#29bcd8)';
            if (statusEl) {
                statusEl.innerText = `Selected: ${n.ssid} — enter password and connect`;
                statusEl.style.color = 'var(--accent-gold)';
            }
        };
        listDiv.appendChild(item);
    });

    resultsDiv.style.display = 'block';

    // Auto-select: prefer last-used camera (recalled from localStorage),
    // fall back to first Sony/DIRECT- network in list.
    let savedSsid = '';
    try { savedSsid = localStorage.getItem('sony_ssid') || ''; } catch (_) {}
    const preferred = savedSsid
        ? data.networks.find(n => n.ssid === savedSsid)
        : null;
    const firstSony = preferred || data.networks.find(n =>
        n.ssid.startsWith('DIRECT-') ||
        n.ssid.toUpperCase().includes('SONY') ||
        n.ssid.toUpperCase().includes('ILCE')
    );
    if (firstSony) {
        const ssidInput = document.getElementById('sony_ssid');
        if (ssidInput) ssidInput.value = firstSony.ssid;
        // Highlight matching item in list
        const idx = data.networks.indexOf(firstSony);
        const items = listDiv.querySelectorAll('div');
        if (items[idx]) items[idx].style.outline = '2px solid var(--accent-teal,#29bcd8)';
        if (statusEl) {
            const label = preferred ? 'Last camera found' : 'Camera found';
            statusEl.innerText = `${label}: ${firstSony.ssid} — enter password and connect`;
            statusEl.style.color = 'var(--accent-green,#3a7)';
        }
    }
}

function connectSony() {
    const ssid = document.getElementById('sony_ssid').value.trim();
    const password = document.getElementById('sony_password').value;
    const ip = document.getElementById('sony_ip').value.trim();
    const statusEl = document.getElementById('sony_status');

    if (!ssid || !password) {
        statusEl.innerText = "ERROR — SSID and password required.";
        statusEl.style.color = "var(--accent-red)";
        return;
    }

    // Persist SSID + password so they're recalled next time.
    // IP is intentionally NOT saved — it's discovered from the wlan1 gateway each time.
    try {
        localStorage.setItem('sony_ssid',     ssid);
        localStorage.setItem('sony_password', password);
        localStorage.removeItem('sony_ip');   // clear any stale hardcoded IP
    } catch (_) {}

    statusEl.innerText = "CONNECTING… joining " + ssid;
    statusEl.style.color = "var(--accent-gold)";

    sendCmd('connect_sony_wifi', { ssid, password, ip });
    log(`Sony: initiating connection to ${ssid} → ${ip}`);
}

function _recallSonyCredentials() {
    try {
        const ssid     = localStorage.getItem('sony_ssid');
        const password = localStorage.getItem('sony_password');
        const ssidEl   = document.getElementById('sony_ssid');
        const pwEl     = document.getElementById('sony_password');
        if (ssid     && ssidEl) ssidEl.value = ssid;
        if (password && pwEl)   pwEl.value   = password;
        // IP is never recalled — always auto-discovered from wlan1 gateway
    } catch (_) {}
}

function disconnectSony() {
    sendCmd('disconnect_sony_wifi');
    const statusEl = document.getElementById('sony_status');
    statusEl.innerText = "DISCONNECTED";
    statusEl.style.color = "var(--text-dim)";
    log("Sony: WiFi dropped.");
}

// Called from WS handler when backend reports Sony status
function updateSonyStatus(status) {
    const statusEl = document.getElementById('sony_status');
    if (!statusEl) return;
    if (status.connected) {
        statusEl.innerText = `CONNECTED — ${status.ip}  Model: ${status.model || '?'}`;
        statusEl.style.color = "var(--accent-green)";
    } else if (status.error) {
        statusEl.innerText = `FAILED — ${status.error}`;
        statusEl.style.color = "var(--accent-red)";
    } else if (status.msg) {
        statusEl.innerText = `⟳ ${status.msg}`;
        statusEl.style.color = "var(--accent-gold)";
    } else {
        statusEl.innerText = "—";
        statusEl.style.color = "var(--text-dim)";
    }
}

// ─── COMPASS CALIBRATION ────────────────────────────────────────────────────
// Track current rig position in motor-odometer space
const calibState = {
    pan_deg: 0.0,  // current pan in degrees (relative to motor zero)
    tilt_deg: 0.0,  // current tilt in degrees
    origin_az: null, // real-world bearing of motor-zero after calibration
    calibrated: false,
};

// Show/hide custom bearing input
document.addEventListener('DOMContentLoaded', () => {
    _recallSonyCredentials();   // restore last-used camera SSID/password

    const bearingSelect = document.getElementById('calib_bearing');
    if (bearingSelect) {
        bearingSelect.addEventListener('change', () => {
            const wrap = document.getElementById('calib_custom_wrap');
            if (wrap) wrap.style.display = bearingSelect.value === 'custom' ? 'block' : 'none';
        });
    }
});

function getSelectedBearing() {
    const sel = document.getElementById('calib_bearing');
    if (!sel) return 90;
    if (sel.value === 'custom') {
        return parseFloat(document.getElementById('calib_custom_deg').value) || 0;
    }
    return parseFloat(sel.value);
}

function nudgeAxis(axis, deg) {
    // Send incremental move to backend — backend executes it and returns new odometer position
    sendCmd('nudge_axis', { axis, deg });
    // Optimistically update local display
    if (axis === 'pan') {
        calibState.pan_deg = Math.round((calibState.pan_deg + deg) * 10) / 10;
    } else {
        calibState.tilt_deg = Math.round((calibState.tilt_deg + deg) * 10) / 10;
    }
    updateCalibReadout();
}

// ── UI Jog buttons — click-to-step / hold-to-move ────────────────────────────
//
// TAP  (pointer held < HOLD_THRESHOLD_MS):
//   Sends nudge_axis → server does a precise timed step and stops automatically.
//   No risk of runaway — server manages the stop internally.
//
// HOLD (pointer held ≥ HOLD_THRESHOLD_MS):
//   Sends ui_nudge_start → motor runs continuously at JOG_SPEED.
//   A keepalive fires every NUDGE_KEEPALIVE_MS so the server watchdog knows
//   we're still holding.  On release, ui_nudge_stop + hw.stop_all_axes() stop
//   the motor immediately.
//
// Safety layers:
//   1. onpointerup / onpointercancel on every button (pointerleave intentionally
//      omitted — hover-off should NOT stop a held nudge, only release does)
//   2. document pointerup listener catches release outside the button
//   3. window blur + visibilitychange stop nudge on tab switch
//   4. socket.onclose / socket.onopen cancel all timers on WS drop/reconnect
//   5. Server 500 ms watchdog kills any ui_nudge with no keepalive

const HOLD_THRESHOLD_MS  = 300;   // tap faster than this → step; slower → continuous
const NUDGE_KEEPALIVE_MS = 150;   // keepalive interval (must be < server 500 ms timeout)

// Per-axis step sizes for tap mode
const JOG_TAP = { pan: 1.0, tilt: 1.0, slider: 5.0 };   // degrees / mm

// Continuous hold speeds
const JOG_SPEED = { pan: 10.0, tilt: 8.0, slider: 40.0 };  // deg/s or mm/s

// Timer state — kept module-level so _stopAllUiNudges can always clean up
const _nudgeHoldGates = {};   // axis → setTimeout id (gate before hold activates)
const _nudgeTimers    = {};   // axis → setInterval id (keepalive while holding)
const _nudgePressTime = {};   // axis → Date.now() when pointer went down
const _nudgePressDir  = {};   // axis → direction (+1 / -1) of current press

function startUiNudge(axis, dir, btn) {
    // Conflict guard: don't start a nudge if the smooth stick/strip is driving this axis
    if ((axis === 'pan' || axis === 'tilt') && joystickActive) return;
    if (axis === 'slider' && sliderStripActive) return;

    _cancelNudgeAxis(axis);   // cancel any prior timers for this axis

    if (btn) btn.classList.add('active');
    _nudgePressTime[axis] = Date.now();
    _nudgePressDir[axis]  = dir;

    // After HOLD_THRESHOLD_MS without release, switch to continuous movement
    _nudgeHoldGates[axis] = setTimeout(() => {
        delete _nudgeHoldGates[axis];
        const speed = JOG_SPEED[axis];
        sendCmd('ui_nudge_start', { axis, dir, speed });
        // Keepalive — server watchdog stops if this stops arriving
        _nudgeTimers[axis] = setInterval(() => {
            sendCmd('ui_nudge_start', { axis, dir, speed });
        }, NUDGE_KEEPALIVE_MS);
    }, HOLD_THRESHOLD_MS);
}

function stopUiNudge(axis, btn) {
    const pressTime = _nudgePressTime[axis] || 0;
    const dir       = _nudgePressDir[axis]  || 1;
    const wasHolding = !!_nudgeTimers[axis];   // setInterval active = we entered hold mode

    _cancelNudgeAxis(axis);
    if (btn) btn.classList.remove('active');
    delete _nudgePressTime[axis];
    delete _nudgePressDir[axis];

    if (wasHolding) {
        // Held long enough — stop continuous movement
        sendCmd('ui_nudge_stop', { axis });
    } else {
        // Quick tap (released before hold threshold) — send a small fixed step
        const deg = dir * JOG_TAP[axis];
        sendCmd('nudge_axis', { axis, deg });
    }
}

function _cancelNudgeAxis(axis) {
    if (_nudgeHoldGates[axis]) { clearTimeout(_nudgeHoldGates[axis]);  delete _nudgeHoldGates[axis]; }
    if (_nudgeTimers[axis])    { clearInterval(_nudgeTimers[axis]);     delete _nudgeTimers[axis]; }
}

// ── Global safety net ─────────────────────────────────────────────────────────
// Fires on pointer release anywhere on the document, tab switch, or WS drop.
function _stopAllUiNudges() {
    const axes = new Set([
        ...Object.keys(_nudgeTimers),
        ...Object.keys(_nudgeHoldGates),
    ]);
    axes.forEach(axis => {
        const wasHolding = !!_nudgeTimers[axis];
        _cancelNudgeAxis(axis);
        if (wasHolding) sendCmd('ui_nudge_stop', { axis });
        delete _nudgePressTime[axis];
        delete _nudgePressDir[axis];
    });
    document.querySelectorAll('.jog-btn').forEach(b => b.classList.remove('active'));
}

document.addEventListener('pointerup',     _stopAllUiNudges);
document.addEventListener('pointercancel', _stopAllUiNudges);
window.addEventListener('blur', _stopAllUiNudges);
window.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'hidden') _stopAllUiNudges();
});

// ── Hardware Reference Zero ───────────────────────────────────────────────────
function hardwareZero() {
    if (!confirm(
        'Zero all motor positions?\n\n' +
        'Park gantry at rail CENTRE, camera LEVEL and PERPENDICULAR to rail first.\n\n' +
        '⚠ Existing soft limits and keyframes will be cleared.\n\n' +
        'This is your absolute position reference for this session.'
    )) return;
    sendCmd('hardware_zero');
}

function handleHardwareZeroed(data) {
    // Update position readouts
    const els_ = els;
    if (els_.valS) els_.valS.innerText = '0.0';
    if (els_.valP) els_.valP.innerText = '0.0';
    if (els_.valT) els_.valT.innerText = '0.0';
    calibState.pan_deg  = 0.0;
    calibState.tilt_deg = 0.0;

    // Update the status box in the hardware reference panel
    const statusEl = document.getElementById('hw_zero_status');
    if (statusEl) {
        const t = new Date().toLocaleTimeString();
        statusEl.style.color = 'var(--accent-green, #3a7)';
        statusEl.innerText = `✓ Zeroed at ${t}  |  s:0.0mm  p:0.0°  t:0.0°`;
    }

    // Make the Zero button look confirmed
    const btn = document.getElementById('hwZeroBtn');
    if (btn) {
        btn.style.background = 'rgba(51,170,80,0.25)';
        btn.style.borderColor = 'var(--accent-green, #3a7)';
        btn.innerText = '✓ Hardware Zeroed';
    }

    updateCalibReadout();
}

function calibrateOrigin() {
    const bearing = getSelectedBearing();
    // Tell backend: "current motor position = this real-world bearing"
    sendCmd('calibrate_origin', {
        bearing_deg: bearing,
        current_pan_deg: calibState.pan_deg,
        current_tilt_deg: calibState.tilt_deg,
    });
    calibState.origin_az = bearing;
    calibState.calibrated = true;

    // Update HG cam_az field to match
    const camAzEl = document.getElementById('hg_cam_az');
    if (camAzEl) camAzEl.value = bearing;

    updateCalibReadout();
    log(`Calibration set: motor-zero → ${bearing}° (${getBearingName(bearing)}). HG az updated.`);
}

function updateCalibReadout() {
    const el = document.getElementById('calib_readout');
    if (!el) return;
    if (calibState.calibrated) {
        const worldPan = ((calibState.origin_az + calibState.pan_deg) % 360 + 360) % 360;
        el.innerText = `CALIBRATED ✓  |  Pan: ${calibState.pan_deg.toFixed(1)}°  Tilt: ${calibState.tilt_deg.toFixed(1)}°  |  World Az: ${worldPan.toFixed(1)}°`;
        el.style.color = "var(--accent-green)";
    } else {
        el.innerText = `NOT CALIBRATED  |  Pan: ${calibState.pan_deg.toFixed(1)}°  Tilt: ${calibState.tilt_deg.toFixed(1)}°`;
        el.style.color = "var(--text-dim)";
    }
}

function getBearingName(deg) {
    const names = { 0: 'N', 45: 'NE', 90: 'E', 135: 'SE', 180: 'S', 225: 'SW', 270: 'W', 315: 'NW' };
    return names[deg] || `${deg}°`;
}


// ─── LOG BUFFER + DRAWER ────────────────────────────────────────────────────

const _logLines = [];
const _MAX_LOG  = 300;
let   _logDrawerOpen = false;

function log(msg) {
    const ts   = new Date().toLocaleTimeString();
    const line = `[${ts}] ${msg}`;

    // Rolling buffer
    _logLines.push(line);
    if (_logLines.length > _MAX_LOG) _logLines.shift();

    // Footer bar (latest message only)
    if (els.debugOverlay) els.debugOverlay.innerText = line;

    // Footer toggle count
    _updateLogToggle();

    // Append to drawer if it's open
    if (_logDrawerOpen) _appendLogEntry(line, true);

    console.log(`[PiSlider] ${msg}`);
}

function _updateLogToggle() {
    const el = document.getElementById('logDrawerToggle');
    if (el) el.textContent = `${_logDrawerOpen ? '▼' : '▲'} LOG (${_logLines.length})`;
}

function _escHtml(s) {
    return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

function _appendLogEntry(line, scroll) {
    const container = document.getElementById('logLinesContainer');
    if (!container) return;
    const div = document.createElement('div');
    div.className = 'log-entry';
    const m = line.match(/^(\[[^\]]+\])\s(.*)$/s);
    if (m) {
        div.innerHTML = `<span class="log-ts">${_escHtml(m[1])}</span> ${_escHtml(m[2])}`;
    } else {
        div.textContent = line;
    }
    container.appendChild(div);
    // Trim displayed list
    while (container.children.length > _MAX_LOG) container.removeChild(container.firstChild);
    if (scroll) container.scrollTop = container.scrollHeight;
}

function toggleLogDrawer() {
    _logDrawerOpen = !_logDrawerOpen;
    const drawer    = document.getElementById('logDrawer');
    const title     = document.getElementById('logDrawerTitle');
    if (!drawer) return;

    if (_logDrawerOpen) {
        drawer.classList.add('open');
        if (title) title.textContent = 'APP LOG';
        // Populate with full history
        const container = document.getElementById('logLinesContainer');
        if (container) {
            container.innerHTML = '';
            _logLines.forEach(l => _appendLogEntry(l, false));
            container.scrollTop = container.scrollHeight;
        }
    } else {
        drawer.classList.remove('open');
    }
    _updateLogToggle();
}

function copyAllLogs(e) {
    e.stopPropagation();
    const text = _logLines.join('\n');
    navigator.clipboard.writeText(text).then(() => {
        const btn = document.getElementById('logCopyBtn');
        if (btn) { btn.textContent = '✓ Copied!'; setTimeout(() => btn.textContent = 'Copy All', 1800); }
    }).catch(() => {
        // Fallback for non-https
        const ta = document.createElement('textarea');
        ta.value = text;
        ta.style.position = 'fixed'; ta.style.opacity = '0';
        document.body.appendChild(ta); ta.select();
        document.execCommand('copy');
        document.body.removeChild(ta);
    });
}

function clearLog(e) {
    e.stopPropagation();
    _logLines.length = 0;
    const container = document.getElementById('logLinesContainer');
    if (container) container.innerHTML = '';
    if (els.debugOverlay) els.debugOverlay.innerText = 'LOG: cleared';
    _updateLogToggle();
}


// ─── CANVAS OVERLAY ENGINE ──────────────────────────────────────────────────
// Handles two overlays drawn on a single canvas:
//   1. LOUPE: draggable circle with 4× magnified crop from the live feed
//   2. MOTION ROI: draggable bounding box for motion detection zone

const overlay = {
    canvas: null,
    ctx: null,
    feed: null,       // the <img> element we sample pixels from
    cw: 0, ch: 0,        // canvas pixel dimensions

    // Loupe state
    loupe: {
        x: 0.5, y: 0.5, // centre as fraction of canvas (0–1)
        r: 100,          // radius in canvas px
        zoom: 4,         // magnification factor
        dragging: false,
        visible: true,
    },

    // Motion ROI state (fractions 0–1)
    roi: {
        x1: 0.25, y1: 0.25,
        x2: 0.75, y2: 0.75,
        visible: false,
        dragging: null,  // null | 'x1y1' | 'x2y2' | 'x1y2' | 'x2y1' | 'body'
        handleR: 10,     // handle hit radius px
        dragStartMx: 0, dragStartMy: 0,
        dragStartRoi: null,
    },
};

function setupCanvasOverlay() {
    overlay.canvas = document.getElementById('overlayCanvas');
    overlay.feed = document.getElementById('mjpegFeed');
    if (!overlay.canvas) return;
    overlay.ctx = overlay.canvas.getContext('2d');

    const container = document.getElementById('feedContainer');

    function resize() {
        const r = container.getBoundingClientRect();
        overlay.canvas.width = r.width;
        overlay.canvas.height = r.height;
        overlay.cw = r.width;
        overlay.ch = r.height;
        // Loupe radius: 18% of the shorter dimension (height in 4:3 landscape)
        overlay.loupe.r = Math.round(Math.min(r.width, r.height) * 0.18);
    }
    resize();
    new ResizeObserver(resize).observe(container);

    // Enable pointer events on canvas for drag handling
    overlay.canvas.style.pointerEvents = 'auto';
    overlay.canvas.style.cursor = 'crosshair';

    overlay.canvas.addEventListener('pointerdown', onOverlayDown);
    overlay.canvas.addEventListener('pointermove', onOverlayMove);
    overlay.canvas.addEventListener('pointerup', onOverlayUp);
    overlay.canvas.addEventListener('pointercancel', onOverlayUp);
    overlay.canvas.addEventListener('dblclick', onOverlayDblClick);

    // Kick off render loop
    requestAnimationFrame(drawOverlay);

    log("Canvas overlay ready — loupe + motion ROI active.");
}

function onOverlayDown(e) {
    const { mx, my } = getMouseFrac(e);
    const l = overlay.loupe;
    const roi = overlay.roi;

    overlay.canvas.setPointerCapture(e.pointerId);

    // Check if inside loupe circle
    const dx = (mx - l.x) * overlay.cw;
    const dy = (my - l.y) * overlay.ch;
    if (Math.sqrt(dx * dx + dy * dy) < l.r) {
        l.dragging = true;
        return;
    }

    // Check ROI handles if visible
    if (roi.visible) {
        const handle = getRoiHandle(mx, my);
        if (handle) {
            roi.dragging = handle;
            roi.dragStartMx = mx; roi.dragStartMy = my;
            roi.dragStartRoi = { ...roi };
            return;
        }
        // Click inside ROI body → move whole box
        if (mx > roi.x1 && mx < roi.x2 && my > roi.y1 && my < roi.y2) {
            roi.dragging = 'body';
            roi.dragStartMx = mx; roi.dragStartMy = my;
            roi.dragStartRoi = { ...roi };
        }
    }
}

function onOverlayMove(e) {
    const { mx, my } = getMouseFrac(e);
    const l = overlay.loupe;
    const roi = overlay.roi;

    if (l.dragging) {
        l.x = Math.max(0, Math.min(1, mx));
        l.y = Math.max(0, Math.min(1, my));
        return;
    }

    if (roi.dragging) {
        const ddx = mx - roi.dragStartMx;
        const ddy = my - roi.dragStartMy;
        const sr = roi.dragStartRoi;
        const MIN = 0.05;

        if (roi.dragging === 'body') {
            const w = sr.x2 - sr.x1, h = sr.y2 - sr.y1;
            roi.x1 = Math.max(0, Math.min(1 - w, sr.x1 + ddx));
            roi.y1 = Math.max(0, Math.min(1 - h, sr.y1 + ddy));
            roi.x2 = roi.x1 + w;
            roi.y2 = roi.y1 + h;
        } else {
            if (roi.dragging.includes('x1')) roi.x1 = Math.max(0, Math.min(roi.x2 - MIN, sr.x1 + ddx));
            if (roi.dragging.includes('x2')) roi.x2 = Math.min(1, Math.max(roi.x1 + MIN, sr.x2 + ddx));
            if (roi.dragging.includes('y1')) roi.y1 = Math.max(0, Math.min(roi.y2 - MIN, sr.y1 + ddy));
            if (roi.dragging.includes('y2')) roi.y2 = Math.min(1, Math.max(roi.y1 + MIN, sr.y2 + ddy));
        }

        // Update cursor
        overlay.canvas.style.cursor = getCursorForHandle(roi.dragging);
        sendMotionSettings();
        updateMotionRoiReadout();
        return;
    }

    // Update cursor based on hover
    if (roi.visible) {
        const h = getRoiHandle(mx, my);
        if (h) { overlay.canvas.style.cursor = getCursorForHandle(h); return; }
        if (mx > roi.x1 && mx < roi.x2 && my > roi.y1 && my < roi.y2) {
            overlay.canvas.style.cursor = 'move'; return;
        }
    }
    const l2 = overlay.loupe;
    const dx = (mx - l2.x) * overlay.cw, dy = (my - l2.y) * overlay.ch;
    overlay.canvas.style.cursor = Math.sqrt(dx * dx + dy * dy) < l2.r ? 'grab' : 'crosshair';
}

function onOverlayUp(e) {
    overlay.loupe.dragging = false;
    overlay.roi.dragging = null;
    overlay.canvas.style.cursor = 'crosshair';
    overlay.canvas.releasePointerCapture(e.pointerId);
}

function onOverlayDblClick(e) {
    // Double-click recenters loupe
    overlay.loupe.x = 0.5;
    overlay.loupe.y = 0.5;
    log("Loupe: recentered.");
}

function getMouseFrac(e) {
    const r = overlay.canvas.getBoundingClientRect();
    return {
        mx: (e.clientX - r.left) / r.width,
        my: (e.clientY - r.top) / r.height,
    };
}

function getRoiHandle(mx, my) {
    const roi = overlay.roi;
    const hr = overlay.roi.handleR / overlay.cw;
    const corners = [
        { name: 'x1y1', x: roi.x1, y: roi.y1 },
        { name: 'x2y1', x: roi.x2, y: roi.y1 },
        { name: 'x1y2', x: roi.x1, y: roi.y2 },
        { name: 'x2y2', x: roi.x2, y: roi.y2 },
    ];
    for (const c of corners) {
        const dx = (mx - c.x) * overlay.cw;
        const dy = (my - c.y) * overlay.ch;
        if (Math.sqrt(dx * dx + dy * dy) < overlay.roi.handleR) return c.name;
    }
    return null;
}

function getCursorForHandle(h) {
    const map = {
        x1y1: 'nw-resize', x2y2: 'se-resize',
        x2y1: 'ne-resize', x1y2: 'sw-resize',
        body: 'move',
    };
    return map[h] || 'crosshair';
}

function drawOverlay() {
    requestAnimationFrame(drawOverlay);
    const ctx = overlay.ctx;
    const cw = overlay.cw, ch = overlay.ch;
    if (!ctx || cw === 0) return;

    ctx.clearRect(0, 0, cw, ch);

    // ── Draw motion ROI ──────────────────────────────────────────────────────
    const roi = overlay.roi;
    if (roi.visible) {
        const rx1 = roi.x1 * cw, ry1 = roi.y1 * ch;
        const rx2 = roi.x2 * cw, ry2 = roi.y2 * ch;

        // Shaded outside
        ctx.fillStyle = 'rgba(0,0,0,0.35)';
        ctx.fillRect(0, 0, cw, ry1);
        ctx.fillRect(0, ry2, cw, ch - ry2);
        ctx.fillRect(0, ry1, rx1, ry2 - ry1);
        ctx.fillRect(rx2, ry1, cw - rx2, ry2 - ry1);

        // Box outline
        ctx.strokeStyle = '#00AAFF';
        ctx.lineWidth = 2;
        ctx.strokeRect(rx1, ry1, rx2 - rx1, ry2 - ry1);

        // Corner handles
        const hr = overlay.roi.handleR;
        ctx.fillStyle = '#00AAFF';
        [[rx1, ry1], [rx2, ry1], [rx1, ry2], [rx2, ry2]].forEach(([hx, hy]) => {
            ctx.beginPath();
            ctx.arc(hx, hy, hr, 0, Math.PI * 2);
            ctx.fill();
        });

        // Label
        ctx.fillStyle = '#00AAFF';
        ctx.font = '11px monospace';
        ctx.fillText('MOTION ZONE', rx1 + 6, ry1 + 16);
    }

    // ── Draw loupe ───────────────────────────────────────────────────────────
    const l = overlay.loupe;
    if (!l.visible) return;

    const lx = l.x * cw;
    const ly = l.y * ch;
    const r = l.r;

    ctx.save();

    // Always fill the loupe background first so it's never transparent
    ctx.beginPath();
    ctx.arc(lx, ly, r, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0,0,0,0.75)';
    ctx.fill();

    // Clip to circle
    ctx.beginPath();
    ctx.arc(lx, ly, r, 0, Math.PI * 2);
    ctx.clip();

    // Draw the high-res crop if available (blob URL — never taints canvas)
    if (loupeCropImage && loupeCropImage.complete && loupeCropImage.naturalWidth > 0) {
        ctx.drawImage(loupeCropImage, lx - r, ly - r, r * 2, r * 2);
    } else {
        // Waiting for first crop — show a dim "loading" indicator
        ctx.fillStyle = 'rgba(0,170,255,0.12)';
        ctx.fillRect(lx - r, ly - r, r * 2, r * 2);
        ctx.fillStyle = 'rgba(0,170,255,0.5)';
        ctx.font = `${Math.round(r * 0.22)}px monospace`;
        ctx.textAlign = 'center';
        ctx.fillText('LOADING…', lx, ly + 6);
        ctx.textAlign = 'left';
    }

    // Crosshair
    ctx.strokeStyle = 'rgba(255,255,255,0.7)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(lx, ly - r); ctx.lineTo(lx, ly + r); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(lx - r, ly); ctx.lineTo(lx + r, ly); ctx.stroke();

    ctx.restore();

    // Loupe border ring (drawn outside clip so it's crisp)
    ctx.beginPath();
    ctx.arc(lx, ly, r, 0, Math.PI * 2);
    ctx.strokeStyle = overlay.roi.visible ? 'rgba(0,170,255,0.5)' : 'rgba(0,170,255,0.9)';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Loupe label
    ctx.fillStyle = 'rgba(0,170,255,0.85)';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${l.zoom}× FOCUS`, lx, ly + r - 10);
    ctx.textAlign = 'left';
}

// Called from setRunState — hide loupe while a non-cinematic sequence runs
function setLoupeVisible(v) {
    // Always respect user toggle — if user hid the loupe, don't show it
    overlay.loupe.visible = v && _loupeUserVisible;
}

// ── Frame rate selector ───────────────────────────────────────────────────────
function setCineFps(fps) {
    sendCmd('set_cine_fps', { value: fps });
    _applyCineFpsUI(fps);
}

function _applyCineFpsUI(fps) {
    [24, 25, 30, 60].forEach(f => {
        const btn = document.getElementById(`fps${f}`);
        if (!btn) return;
        const active = f === fps;
        btn.style.background = active ? 'var(--accent-teal)' : 'none';
        btn.style.borderColor = active ? 'var(--accent-teal)' : '#333';
        btn.style.color = active ? '#000' : '#555';
    });
}
const _ORIENT_ICONS = {
    landscape: '⬜',
    portrait_cw: '↻',
    portrait_ccw: '↺',
    inverted: '⬛',
};

function setCameraOrientation(orient) {
    sendCmd('set_camera_orientation', { value: orient });
    _applyOrientationUI(orient);
}

function _applyOrientationUI(orient) {
    ['landscape', 'portrait_cw', 'portrait_ccw', 'inverted'].forEach(o => {
        const btn = document.getElementById('orient' + {
            landscape: 'Land', portrait_cw: 'CW',
            portrait_ccw: 'CCW', inverted: 'Flip'
        }[o]);
        if (!btn) return;
        const active = o === orient;
        btn.style.background = active ? 'var(--accent-teal)' : 'none';
        btn.style.borderColor = active ? 'var(--accent-teal)' : '#444';
        btn.style.color = active ? '#000' : '#666';
    });
}

function handleCameraOrientation(data) {
    _applyOrientationUI(data.value);
}
function toggleLoupeVisibility() {
    _loupeUserVisible = !_loupeUserVisible;
    overlay.loupe.visible = _loupeUserVisible;
    const btn = document.getElementById('loupeToggleBtn');
    if (btn) {
        btn.style.opacity = _loupeUserVisible ? '1' : '0.4';
        btn.style.borderColor = _loupeUserVisible ? 'var(--accent-teal)' : '#444';
        btn.style.color = _loupeUserVisible ? 'var(--accent-teal)' : '#555';
        btn.title = _loupeUserVisible ? 'Hide focus loupe' : 'Show focus loupe';
    }
}

// Called from onTriggerModeChange
function setRoiVisible(v) {
    overlay.roi.visible = v;
    // When motion mode enabled and non-PiCam active, show a note
    const note = document.getElementById('motion_camera_note');
    if (note) note.style.display = v ? 'block' : 'none';
}

function updateMotionRoiReadout() {
    const roi = overlay.roi;
    const el = document.getElementById('motion_roi_readout');
    if (el) el.innerText = `Zone: x ${(roi.x1 * 100).toFixed(0)}%–${(roi.x2 * 100).toFixed(0)}%,  y ${(roi.y1 * 100).toFixed(0)}%–${(roi.y2 * 100).toFixed(0)}%`;
}

// Restore ROI from server data
function setRoiFromData(roi) {
    if (!roi || roi.length < 4) return;
    overlay.roi.x1 = roi[0]; overlay.roi.y1 = roi[1];
    overlay.roi.x2 = roi[2]; overlay.roi.y2 = roi[3];
    updateMotionRoiReadout();
}


// ═══════════════════════════════════════════════════════════════════════════════
// DISK SPACE DISPLAY + ALERTS
// ═══════════════════════════════════════════════════════════════════════════════

async function updateDiskSpace() {
    try {
        const resp = await fetch('/disk_info');
        const d = await resp.json();
        if (d.error) return;
        const freeGB = (d.free / 1e9).toFixed(1);
        const totalGB = (d.total / 1e9).toFixed(1);
        const pct = Math.round(d.used / d.total * 100);
        const cam = document.getElementById('camera_select')?.value || 'picam';
        const framesEst = Math.floor(d.free / ((cam === 'sony' ? 30 : 25) * 1e6));

        const el = document.getElementById('disk_free_label');
        if (!el) return;
        el.innerHTML = `${freeGB} GB free of ${totalGB} GB (${pct}% used) — ~<b>${framesEst.toLocaleString()}</b> frames of space`;

        const row = document.getElementById('disk_space_row');
        if (row) {
            row.classList.remove('disk-ok', 'disk-warn', 'disk-crit');
            if (pct > 90) row.classList.add('disk-crit');
            else if (pct > 75) row.classList.add('disk-warn');
            else row.classList.add('disk-ok');
        }

        // Pre-flight warning
        const total = parseInt(document.getElementById('total_frames')?.value) || 0;
        const warnEl = document.getElementById('disk_preflight_warn');
        if (warnEl) {
            const show = total > 0 && framesEst < total;
            warnEl.style.display = show ? 'block' : 'none';
            if (show) warnEl.textContent =
                `⚠ Only ~${framesEst.toLocaleString()} frames fit on disk — sequence of ${total} may fail.`;
        }
    } catch (_) { }
}

let _diskPollTimer = null;
function startDiskPolling() {
    updateDiskSpace();
    if (!_diskPollTimer) _diskPollTimer = setInterval(updateDiskSpace, 15000);
}

function showDiskAlert(msg, isFull) {
    document.getElementById('diskAlertBanner')?.remove();
    const banner = document.createElement('div');
    banner.id = 'diskAlertBanner';
    banner.className = 'disk-alert-banner';
    banner.innerHTML = `
        <div style="font-size:1.1rem; margin-bottom:4px;">${isFull ? '⛔ DISK FULL' : '⚠ LOW DISK SPACE'}</div>
        <div style="font-size:0.8rem; opacity:0.9;">${msg}</div>
        <button onclick="document.getElementById('diskAlertBanner').remove()">Dismiss</button>
    `;
    document.body.appendChild(banner);
    log(msg);
    if (!isFull) setTimeout(() => { banner?.remove(); }, 30000);
}

function _showKickedBanner(msg) {
    // Full-screen overlay — makes it impossible to accidentally use a displaced tab
    const existing = document.getElementById('kickedOverlay');
    if (existing) existing.remove();
    const overlay = document.createElement('div');
    overlay.id = 'kickedOverlay';
    overlay.style.cssText = `
        position: fixed; inset: 0; z-index: 9999;
        background: rgba(0,0,0,0.88);
        display: flex; flex-direction: column;
        align-items: center; justify-content: center;
        color: var(--accent-gold, #f0a500);
        font-family: monospace; text-align: center; gap: 16px;
        backdrop-filter: blur(4px);
    `;
    overlay.innerHTML = `
        <div style="font-size:2rem;">⚠</div>
        <div style="font-size:1.1rem; font-weight:bold;">TAB REPLACED</div>
        <div style="font-size:0.85rem; max-width:360px; color:#ccc; line-height:1.6;">${msg}</div>
        <button onclick="window.location.reload()"
                style="margin-top:8px; padding:10px 28px; background:var(--accent-gold,#f0a500);
                       color:#000; border:none; border-radius:6px; font-weight:bold;
                       cursor:pointer; font-size:0.9rem;">
            Take Control Back
        </button>
    `;
    document.body.appendChild(overlay);
}

function handleDiskFull(data) { showDiskAlert(data.msg, true); updateDiskSpace(); }
function handleDiskWarn(data) { showDiskAlert(data.msg, false); updateDiskSpace(); }

function handleSonyUsbStatus(data) {
    const statusEl = document.getElementById('sony_usb_status');
    if (!statusEl) return;
    if (data.found) {
        statusEl.innerHTML = `✅ <b>${data.model}</b> detected on <code>${data.port}</code> — ready to shoot.`;
        statusEl.style.color = 'var(--accent-green, #3a7)';
        log(`Sony USB: ${data.model} found on ${data.port}.`);
    } else {
        const hint = data.port ? ` (${data.port})` : '';
        statusEl.innerHTML = `❌ No Sony camera found${hint}. Check USB cable, PC&nbsp;Remote mode, and USB&nbsp;Power&nbsp;Supply&nbsp;OFF.`;
        statusEl.style.color = 'var(--accent-red, #c44)';
        log(`Sony USB: no camera detected${hint}.`);
    }
}

function handleSonyStorageWarn(data) {
    // Show a dismissible warning banner above the run button (same style as disk warn)
    const pct = data.remaining > 0
        ? Math.round((data.remaining / data.needed) * 100)
        : 0;
    const msg = `📷 Sony card: ${data.remaining} shots left, need ${data.needed} — ` +
                `short by ${data.short_by} (${pct}% capacity). ` +
                `Swap card or reduce frame count.`;
    log(`⚠ ${msg}`);
    showDiskAlert(msg, false);   // reuse disk-warn banner (amber/yellow)
}

function handleSonyRecordError(data) {
    // Show a prominent dismissible banner — record errors are easy to miss in the log
    document.getElementById('sonyRecErrBanner')?.remove();
    const banner = document.createElement('div');
    banner.id = 'sonyRecErrBanner';
    banner.className = 'disk-alert-banner';   // reuse amber banner style
    banner.innerHTML = `
        <div style="font-size:1.1rem; margin-bottom:4px;">🎬 CANNOT START RECORDING</div>
        <div style="font-size:0.85rem; opacity:0.9;">${data.msg}</div>
        <button onclick="document.getElementById('sonyRecErrBanner').remove()">Dismiss</button>
    `;
    document.body.appendChild(banner);
    setTimeout(() => { document.getElementById('sonyRecErrBanner')?.remove(); }, 20000);
}


// ═══════════════════════════════════════════════════════════════════════════════
// MACRO MODE
// ═══════════════════════════════════════════════════════════════════════════════

let _macroMode = 'scan';   // 'scan' | 'art'
let _macroRotMode = 'full';   // 'full' | 'range'
let _macroTiltMode = 'full';   // 'full' | 'limited' (for geodesic 2D grid)
let _macroRailStart = null;
let _macroRailEnd = null;
let _macroRailStartSteps = null;  // ← NEW: Actual step position for accurate macro movement
let _macroRailEndSteps = null;    // ← NEW
let _macroRotStart = null;
let _macroRotEnd = null;
let _macroTiltStart = null;
let _macroTiltEnd = null;
let _macroLensProfiles = {};

// ── Sub-mode toggle ──────────────────────────────────────────────────────────
function setMacroMode(mode) {
    _macroMode = mode;
    document.getElementById('macroModeScan').classList.toggle('active', mode === 'scan');
    document.getElementById('macroModeArt').classList.toggle('active', mode === 'art');

    // Show spacing curve only in Art mode; force 'even' in Scan mode
    const easingRow = document.getElementById('macro_easing_row');
    const easingSelect = document.getElementById('macro_rotation_easing');
    if (easingRow) easingRow.style.display = mode === 'art' ? '' : 'none';
    if (easingSelect) easingSelect.value = 'even';  // Always start with 'even'

    // Show stereo section only in Art mode; force disabled in Scan mode
    const stereoSection = document.getElementById('macro_stereo_section');
    const stereoCheckbox = document.getElementById('macro_stereo_enabled');
    if (stereoSection) stereoSection.style.display = mode === 'art' ? '' : 'none';
    if (stereoCheckbox) stereoCheckbox.checked = false;  // Always disabled in Scan

    // Show Motion Path panel only in Art mode (for keyframe programming)
    const motionPathPanel = document.getElementById('motion_path_panel');
    if (motionPathPanel) motionPathPanel.style.display = mode === 'art' ? '' : 'none';

    macroCalc();
}

// ── Rotation mode (full 360 vs range) ────────────────────────────────────────
function setRotationMode(mode) {
    _macroRotMode = mode;
    document.getElementById('macroRotFull').classList.toggle('active', mode === 'full');
    document.getElementById('macroRotRange').classList.toggle('active', mode === 'range');
    const rc = document.getElementById('macro_rotation_range_controls');
    if (rc) rc.style.display = mode === 'range' ? '' : 'none';
    macroCalc();
}

// ── Tilt mode (full 360 vs limited by soft limits) ─────────────────────────────
function setTiltMode(mode) {
    _macroTiltMode = mode;
    document.getElementById('macroTiltFull').classList.toggle('active', mode === 'full');
    document.getElementById('macroTiltLimited').classList.toggle('active', mode === 'limited');
    const tc = document.getElementById('macro_tilt_range_controls');
    const tl = document.getElementById('macro_tilt_limits');
    if (tc) tc.style.display = mode === 'limited' ? '' : 'none';
    if (tl) tl.style.display = mode === 'limited' ? '' : 'none';  // Show limits only in limited mode
    macroCalc();
}

// (toggleAuxAxis removed — tilt axis now uses setTiltMode() with full/limited toggle)

// ── Soft limits ───────────────────────────────────────────────────────────────
function sendMacroSoftLimits() {
    sendCmd('macro_set_soft_limits', {
        rail_min: parseFloat(document.getElementById('macro_rail_soft_min')?.value ?? -999),
        rail_max: parseFloat(document.getElementById('macro_rail_soft_max')?.value ?? 999),
        pan_min: -360, pan_max: 360,   // rotation stage — wide range
        tilt_min: parseFloat(document.getElementById('macro_tilt_soft_min')?.value ?? -90),
        tilt_max: parseFloat(document.getElementById('macro_tilt_soft_max')?.value ?? 90),
    });
}

// ── Macro Home/Calibration ──────────────────────────────────────────────────────
function macroGoHome() {
    sendCmd('macro_go_home', {});
    log('⏳ Moving to home position: pan=0°, tilt=0°');
}

// ── Easing curves initialization ────────────────────────────────────────────────
function requestMacroEasingCurves() {
    sendCmd('macro_get_easing_curves', {});
}

function handleMacroEasingCurves(data) {
    const curves = data.curves || [];
    const select = document.getElementById('macro_rotation_easing');
    if (!select) return;

    // Store current value
    const currentValue = select.value;

    // Clear and repopulate
    select.innerHTML = '';
    curves.forEach(curve => {
        const opt = document.createElement('option');
        opt.value = curve;
        opt.innerText = curve.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
        select.appendChild(opt);
    });

    // Restore value if it exists, else set to 'even'
    if (curves.includes(currentValue)) {
        select.value = currentValue;
    } else if (curves.includes('even')) {
        select.value = 'even';
    }

    log(`[Macro] Loaded ${curves.length} easing curves`);
}

function macroComputeGrid() {
    // Get current pan range from mode
    let panMin = -90, panMax = 90;
    if (_macroRotMode === 'range' && _macroRotStart !== null && _macroRotEnd !== null) {
        panMin = Math.min(_macroRotStart, _macroRotEnd);
        panMax = Math.max(_macroRotStart, _macroRotEnd);
    } else if (_macroRotMode === 'full') {
        panMin = -180;
        panMax = 180;
    }

    // Get tilt range from mode (for geodesic 2D grid)
    let tiltMin = -90, tiltMax = 90;
    const tiltSoftMin = parseFloat(document.getElementById('macro_tilt_soft_min')?.value ?? -90);
    const tiltSoftMax = parseFloat(document.getElementById('macro_tilt_soft_max')?.value ?? 90);

    if (_macroTiltMode === 'limited' && _macroTiltStart !== null && _macroTiltEnd !== null) {
        tiltMin = Math.min(_macroTiltStart, _macroTiltEnd);
        tiltMax = Math.max(_macroTiltStart, _macroTiltEnd);
    } else if (_macroTiltMode === 'limited') {
        // Use soft limits if no custom start/end set
        tiltMin = tiltSoftMin;
        tiltMax = tiltSoftMax;
    } else if (_macroTiltMode === 'full') {
        tiltMin = -180;
        tiltMax = 180;
    }

    const totalStacks = parseInt(document.getElementById('macro_num_stacks')?.value || 36);

    sendCmd('macro_compute_grid', {
        total_stacks: totalStacks,
        pan_min: panMin,
        pan_max: panMax,
        pan_mode: _macroRotMode,
        tilt_min: tiltMin,
        tilt_max: tiltMax,
        tilt_mode: _macroTiltMode
    });
}

function handleMacroGridComputed(data) {
    const panCols = data.pan_cols || 1;
    const tiltRows = data.tilt_rows || 1;
    const totalActual = data.total_actual || 1;
    const msg = `${panCols} cols × ${tiltRows} rows = ${totalActual} stacks`;

    const el = document.getElementById('macro_grid_result');
    if (el) el.innerText = msg;

    log(`[Macro Grid] ${msg}`);
}

// ── Live calculation ──────────────────────────────────────────────────────────
function macroCalc() {
    const imagesPerStack = parseInt(document.getElementById('macro_images_per_stack')?.value || 9);
    const numStacks = parseInt(document.getElementById('macro_num_stacks')?.value || 36);
    const stereoEnabled = document.getElementById('macro_stereo_enabled')?.checked || false;
    const stereoMultiplier = stereoEnabled ? 2 : 1;
    const slotA = document.getElementById('macro_slot_a_enabled')?.checked ? 1 : 0;
    const slotB = document.getElementById('macro_slot_b_enabled')?.checked ? 1 : 0;
    const slots = slotA + slotB;

    // Calculate travel distance: 2mm per motor step, (N-1) steps between N images
    let travelMm = 0;
    if (imagesPerStack > 0) {
        travelMm = (imagesPerStack - 1) * 2;  // 2mm pitch lead screw
    }

    // Update total travel display
    const ttd = document.getElementById('macro_travel_total_mm');
    if (ttd) ttd.value = travelMm > 0 ? travelMm.toFixed(1) : '—';

    // Auto-compute recommended stacks based on coverage area
    let panRange = 360;  // degrees
    if (_macroRotMode === 'range' && _macroRotStart !== null && _macroRotEnd !== null) {
        panRange = Math.abs(_macroRotEnd - _macroRotStart);
    }

    // Recommended stacks: integrate actual surface area covered, accounting for pan axis tilt.
    //
    // The geodesic weight for each tilt row is |cos(tilt_rad + alpha)| where
    //   alpha = PI/2 - panAxisTilt_rad   (converts UI angle to radians-from-vertical)
    // The integral over the tilt range gives the true spherical surface area fraction:
    //   ∫ |cos(φ + alpha)| dφ  from tiltMin to tiltMax
    //   = |sin(tiltMax_rad + alpha) - sin(tiltMin_rad + alpha)|
    // Normalised to the full sphere (vertical axis, -90→90 tilt = 2.0):
    //   coverageFraction = (panRange/360) × |sin(tiltMax_rad + alpha) − sin(tiltMin_rad + alpha)| / 2
    //
    // Reference: 72 stacks for full sphere coverage (empirically chosen as minimum for
    // COLMAP to find sufficient feature overlap). Scales proportionally for partial coverage.
    const panAxisTiltDeg = parseFloat(document.getElementById('macro_rot_axis_angle')?.value ?? 90);
    const alpha = Math.PI / 2 - panAxisTiltDeg * Math.PI / 180;
    let tiltMin = -90, tiltMax = 90;
    if (_macroTiltMode === 'limited') {
        tiltMin = parseFloat(document.getElementById('macro_tilt_soft_min')?.value ?? -90);
        tiltMax = parseFloat(document.getElementById('macro_tilt_soft_max')?.value ?? 90);
    }
    const tiltMinRad = tiltMin * Math.PI / 180;
    const tiltMaxRad = tiltMax * Math.PI / 180;
    const sphericalCoverage = Math.abs(Math.sin(tiltMaxRad + alpha) - Math.sin(tiltMinRad + alpha));
    const coverageFraction = (panRange / 360) * sphericalCoverage / 2;
    const recommendedStacks = Math.max(4, Math.round(coverageFraction * 72));
    const rs = document.getElementById('macro_recommended_stacks');
    if (rs) rs.innerText = recommendedStacks;

    // Summary calculations
    const totalImages = imagesPerStack * numStacks * stereoMultiplier * Math.max(1, slots);
    const storageGb = (totalImages * 25 / 1024).toFixed(2);

    const sf = document.getElementById('macro_sum_frames');
    const ss = document.getElementById('macro_sum_stacks');
    const si = document.getElementById('macro_sum_images');
    const sg = document.getElementById('macro_sum_storage');
    if (sf) sf.innerText = imagesPerStack || '—';
    if (ss) ss.innerText = stereoEnabled ? `${numStacks} × 2 (stereo)` : numStacks;
    if (si) si.innerText = totalImages || '—';
    if (sg) sg.innerText = storageGb || '—';
}

// ── Position markers ─────────────────────────────────────────────────────────
function handleMacroRailMark(data) {
    if (data.which === 'start') {
        _macroRailStart = data.mm;
        _macroRailStartSteps = data.steps;  // ← Store step position
        const el = document.getElementById('macro_rail_start_disp');
        if (el) el.innerText = data.mm.toFixed(3);
        log(`Rail start: ${data.mm.toFixed(3)}mm (${data.steps} steps)`);
    } else {
        _macroRailEnd = data.mm;
        _macroRailEndSteps = data.steps;  // ← Store step position
        const el = document.getElementById('macro_rail_end_disp');
        if (el) el.innerText = data.mm.toFixed(3);
        log(`Rail end: ${data.mm.toFixed(3)}mm (${data.steps} steps)`);
    }
    // Update travel display
    if (_macroRailStart !== null && _macroRailEnd !== null) {
        const travel = Math.abs(_macroRailEnd - _macroRailStart);
        const el = document.getElementById('macro_rail_travel_disp');
        if (el) el.innerText = travel.toFixed(3);
    }
    macroCalc();
    log(`Rail ${data.which} set: ${data.mm.toFixed(3)} mm`);
}

function handleMacroRotMark(data) {
    if (data.which === 'start') {
        _macroRotStart = data.deg;
        const el = document.getElementById('macro_rot_start_disp');
        if (el) el.innerText = data.deg.toFixed(1);
    } else {
        _macroRotEnd = data.deg;
        const el = document.getElementById('macro_rot_end_disp');
        if (el) el.innerText = data.deg.toFixed(1);
    }
    log(`Rotation ${data.which} set: ${data.deg.toFixed(1)}°`);
    macroCalc();
}

function handleMacroTiltMark(data) {
    if (data.which === 'start') {
        _macroTiltStart = data.deg;
        const el = document.getElementById('macro_tilt_start_disp');
        if (el) el.innerText = data.deg.toFixed(1);
    } else {
        _macroTiltEnd = data.deg;
        const el = document.getElementById('macro_tilt_end_disp');
        if (el) el.innerText = data.deg.toFixed(1);
    }
    log(`Tilt ${data.which} set: ${data.deg.toFixed(1)}°`);
    macroCalc();
}

function handleMacroAuxMark(data) {
    if (data.which === 'start') {
        _macroAuxStart = data.deg;
        const el = document.getElementById('macro_aux_start_disp');
        if (el) el.innerText = data.deg.toFixed(1);
    } else {
        _macroAuxEnd = data.deg;
        const el = document.getElementById('macro_aux_end_disp');
        if (el) el.innerText = data.deg.toFixed(1);
    }
    log(`Aux ${data.which} set: ${data.deg.toFixed(1)}°`);
}

// ── Progress updates ─────────────────────────────────────────────────────────
function handleMacroProgress(data) {
    // Force panel visible with explicit 'block' — avoids inheriting display:none
    // from a collapsed parent section when the user is viewing another mode.
    const panel = document.getElementById('macro_progress_panel');
    if (panel) panel.style.display = 'block';

    const set = (id, val) => { const el = document.getElementById(id); if (el) el.innerText = val; };
    set('macro_prog_stack',        data.stack        ?? '—');
    set('macro_prog_total_stacks', data.total_stacks ?? '—');
    // Use nullish coalescing so frame=0 still shows '0' rather than '—'
    set('macro_prog_frame',        data.frame        ?? '—');
    set('macro_prog_total_frames', data.total_frames ?? '—');
    if (data.rotation_deg !== undefined) set('macro_prog_rot',  data.rotation_deg.toFixed(1));
    if (data.pan_deg      !== undefined) set('macro_prog_rot',  data.pan_deg.toFixed(1));
    if (data.rail_mm      !== undefined) set('macro_prog_rail', data.rail_mm.toFixed(3));
    if (data.msg) set('macro_prog_msg', data.msg);

    // Progress bar — (completed stacks × frames) + current frame
    if (data.total_stacks && data.total_frames) {
        const done  = Math.max(0, (data.stack - 1)) * data.total_frames + (data.frame || 0);
        const total = data.total_stacks * data.total_frames;
        const pct   = Math.min(100, Math.round(done / total * 100));
        const bar   = document.getElementById('macro_prog_bar');
        if (bar) bar.style.width = pct + '%';
    }
}

function handleMacroStackComplete(data) {
    const set = (id, val) => { const el = document.getElementById(id); if (el) el.innerText = val; };
    // After a stack finishes, frame count = total (all frames done for this stack)
    if (data.frame_count && data.total_stacks) {
        set('macro_prog_frame',        data.frame_count);
        set('macro_prog_total_frames', data.frame_count);
    }
    log(`✓ Stack ${data.stack}/${data.total_stacks} complete — rot ${data.rotation_deg?.toFixed(1)}°`);
}

function handleMacroDone(data) {
    const panel = document.getElementById('macro_progress_panel');
    if (panel) panel.style.display = 'none';
    const bar = document.getElementById('macro_prog_bar');
    if (bar) bar.style.width = '0%';
    log(data.msg || (data.interrupted ? '⚠ Macro stopped.' : '✓ Macro sequence complete.'));
}

// ── Lens profiles ─────────────────────────────────────────────────────────────
function macroStoreLensProfile() {
    const name = document.getElementById('macro_lens_name')?.value?.trim();
    if (!name || name === 'unknown') { log('Enter a lens name before saving.'); return; }
    const profile = _readLensProfile();
    sendCmd('macro_store_lens_profile', { name, profile });
    log(`Lens profile '${name}' saved.`);
}

function _readLensProfile() {
    return {
        name: document.getElementById('macro_lens_name')?.value || 'unknown',
        lens_type: document.getElementById('macro_lens_type')?.value || 'macro',
        magnification: parseFloat(document.getElementById('macro_lens_mag')?.value || 1),
        working_distance_mm: parseFloat(document.getElementById('macro_lens_wd')?.value || 0),
        notes: '',
    };
}

function macroLoadLensProfile(name) {
    if (!name || !_macroLensProfiles[name]) return;
    const p = _macroLensProfiles[name];
    const set = (id, val) => { const el = document.getElementById(id); if (el) el.value = val; };
    set('macro_lens_name', p.name || '');
    set('macro_lens_type', p.lens_type || 'macro');
    set('macro_lens_mag', p.magnification || 1);
    set('macro_lens_wd', p.working_distance_mm || 0);
    log(`Lens profile '${name}' loaded.`);
}

function handleMacroLensProfiles(data) {
    _macroLensProfiles = data.profiles || {};
    const sel = document.getElementById('macro_lens_profile_select');
    if (!sel) return;
    sel.innerHTML = '<option value="">— Load saved —</option>';
    Object.keys(_macroLensProfiles).forEach(name => {
        const opt = document.createElement('option');
        opt.value = name; opt.innerText = name;
        sel.appendChild(opt);
    });
}

// ── Start sequence ────────────────────────────────────────────────────────────
function macroStart() {
    if (isRunning) { log('Already running.'); return; }
    if (_macroRailStart === null || _macroRailEnd === null) {
        log('⚠ Set rail start and end positions before starting.');
        return;
    }

    const stereoEnabled = document.getElementById('macro_stereo_enabled')?.checked || false;
    const panAxisAngle = parseFloat(document.getElementById('macro_rot_axis_angle')?.value || 90);

    // Warn if stereo is enabled but pan axis is not vertical
    if (stereoEnabled && Math.abs(panAxisAngle - 90) > 5) {
        log(`⚠️ WARNING: Stereo enabled but pan axis angle = ${panAxisAngle}°. ` +
            `Stereo 3D works best when pan axis is vertical (90°). ` +
            `For 45° tilted axis, disable stereo or reconfigure hardware.`);
    }

    // Tilt is always enabled for geodesic 2D grid in macro mode
    const tiltStart = _macroTiltStart ?? (
        _macroTiltMode === 'limited'
            ? parseFloat(document.getElementById('macro_tilt_soft_min')?.value ?? -90)
            : -180
    );
    const tiltEnd = _macroTiltEnd ?? (
        _macroTiltMode === 'limited'
            ? parseFloat(document.getElementById('macro_tilt_soft_max')?.value ?? 90)
            : 180
    );

    const payload = {
        project_name: document.getElementById('macro_project_name')?.value || 'macro_project',
        orbit_label: document.getElementById('macro_orbit_label')?.value || 'orbit_001',
        session_mode: _macroMode,
        scan_type: 'orbit',  // orbit = pan-rotation sweep with focus stacks (uses num_stacks)
        grid_snake: true,  // Serpentine (boustrophedon) routing for COLMAP adjacent images
        rail_start_mm: _macroRailStart,
        rail_end_mm: _macroRailEnd,
        rail_start_steps: _macroRailStartSteps,  // ← NEW: Actual step positions
        rail_end_steps: _macroRailEndSteps,      // ← NEW
        images_per_stack: parseInt(document.getElementById('macro_images_per_stack')?.value || 9),
        rail_soft_min: parseFloat(document.getElementById('macro_rail_soft_min')?.value ?? -999),
        rail_soft_max: parseFloat(document.getElementById('macro_rail_soft_max')?.value ?? 999),
        rotation_mode: _macroRotMode,
        rotation_start_deg: _macroRotStart ?? 0,
        rotation_end_deg: _macroRotEnd ?? 360,
        num_stacks: parseInt(document.getElementById('macro_num_stacks')?.value || 36),
        rotation_easing: document.getElementById('macro_rotation_easing')?.value || 'even',
        rotation_axis_angle_deg: parseFloat(document.getElementById('macro_rot_axis_angle')?.value || 90),
        rotation_axis_description: document.getElementById('macro_rot_axis_desc')?.value || 'vertical',
        stereo_enabled: document.getElementById('macro_stereo_enabled')?.checked || false,
        stereo_offset_deg: parseFloat(document.getElementById('macro_stereo_offset_deg')?.value || 3.0),
        aux_enabled: true,  // Always enabled for geodesic 2D grid
        aux_label: 'tilt',  // Default label for tilt axis
        aux_start_deg: tiltStart,
        aux_end_deg: tiltEnd,
        aux_easing: 'even',  // Always even distribution for scan mode
        tilt_mode: _macroTiltMode,  // 'full' | 'limited'
        vibe_delay_s: parseFloat(document.getElementById('macro_vibe_delay')?.value || 0.5),
        exp_margin_s: parseFloat(document.getElementById('macro_exp_margin')?.value || 0.2),
        lens_profile: _readLensProfile(),
        slots: [
            {
                id: 'slot_A',
                label: document.getElementById('macro_slot_a_label')?.value || 'diffuse',
                enabled: document.getElementById('macro_slot_a_enabled')?.checked ?? true,
                relay1: document.getElementById('macro_slot_a_relay1')?.checked ?? false,
                relay2: document.getElementById('macro_slot_a_relay2')?.checked ?? false,
                relay_settle_ms: parseInt(document.getElementById('macro_slot_a_settle')?.value || 0),
                relay_release_ms: parseInt(document.getElementById('macro_slot_a_release')?.value || 0),
                iso: parseInt(document.getElementById('macro_slot_a_iso')?.value || 400),
                shutter_s: parseFloat(document.getElementById('macro_slot_a_shutter')?.value || 0.008),
                kelvin: parseInt(document.getElementById('macro_slot_a_kelvin')?.value || 5500),
                ae: document.getElementById('macro_slot_a_ae')?.checked ?? false,
                awb: false,
            },
            {
                id: 'slot_B',
                label: document.getElementById('macro_slot_b_label')?.value || 'laser',
                enabled: document.getElementById('macro_slot_b_enabled')?.checked ?? false,
                relay1: document.getElementById('macro_slot_b_relay1')?.checked ?? false,
                relay2: document.getElementById('macro_slot_b_relay2')?.checked ?? true,
                relay_settle_ms: parseInt(document.getElementById('macro_slot_b_settle')?.value || 250),
                relay_release_ms: parseInt(document.getElementById('macro_slot_b_release')?.value || 0),
                iso: parseInt(document.getElementById('macro_slot_b_iso')?.value || 200),
                shutter_s: parseFloat(document.getElementById('macro_slot_b_shutter')?.value || 0.033),
                kelvin: parseInt(document.getElementById('macro_slot_b_kelvin')?.value || 5500),
                ae: document.getElementById('macro_slot_b_ae')?.checked ?? false,
                awb: false,
            }
        ],
    };

    const stereoNote = payload.stereo_enabled ? ` + STEREO (${payload.stereo_offset_deg}° offset)` : '';
    const travelMm = (payload.rail_end_mm - payload.rail_start_mm).toFixed(1);
    const imagesPerStack = payload.images_per_stack;
    log(`Starting macro: ${payload.num_stacks} stacks × ${imagesPerStack} images (${travelMm}mm travel)${stereoNote}`);
    setRunState(true);
    sendCmd('macro_start', payload);
}


// ═══════════════════════════════════════════════════════════════════════════════
// CINEMATIC MODE
// ═══════════════════════════════════════════════════════════════════════════════

let _cineSubMode = 'live';
let _cineInertiaRunning = false;
let _cineRecording = false;
let _cineRecordTimer = null;
let _cineRecordStart = null;
let _cineKeyframes = [];
let _cineLimits = {};
let _cineOrigin = null;

// Path planning state (mirrors server ProgrammedMove fields)
let _pathMode       = 'linear';
let _globalEasing   = 'cycloid';
let _catmullTension = 0.5;
let _pathMinDuration = 0.0;   // minimum safe cinematic duration (from server)
let _pathMinAxis     = '';    // axis that limits the minimum duration

// ── Sub-mode toggle ──────────────────────────────────────────────────────────
function setCineSubMode(mode) {
    _cineSubMode = mode;
    document.getElementById('cineModeLive').classList.toggle('active', mode === 'live');
    document.getElementById('cineModeProg').classList.toggle('active', mode === 'programmed');
    document.getElementById('cine_live_panel').style.display = mode === 'live' ? '' : 'none';
    document.getElementById('cine_prog_panel').style.display = mode === 'programmed' ? '' : 'none';
    sendCmd('cinematic_set_mode', { value: mode });
}

// ── Soft limits ──────────────────────────────────────────────────────────────
function calibrateLimit(axis, end) {
    sendCmd('cinematic_calibrate_limit', { axis, end });
    log(`Soft limit: ${axis} ${end} set at current position.`);
}

function sendInversion(axis) {
    const el = document.getElementById(`side_${axis}_inv`);
    if (!el) return;
    sendCmd('set_inversion', { axis: axis, inverted: el.checked });
}

function handleInversionsUpdated(data) {
    ['slider', 'pan', 'tilt'].forEach(ax => {
        if (data[ax] !== undefined) {
            const el = document.getElementById(`side_${ax}_inv`);
            if (el) el.checked = data[ax];
        }
    });
}

function handleCineLimits(data) {
    _cineLimits = data.limits || {};
    _updateLimitDisplay();
}

function _updateLimitDisplay() {
    const CAL_LABELS = ['uncalibrated — crawl only', 'one end set — half speed', '✓ both ends — full speed'];
    const SPEED_LABELS = ['CRAWL ONLY', 'HALF SPEED', 'FULL SPEED'];
    const SPEED_COLORS = ['#cc4400', '#ccaa00', '#00cc66'];

    let maxCal = 2;
    ['slider', 'pan', 'tilt'].forEach(axis => {
        const ax = _cineLimits[axis];
        if (!ax) return;
        const cal = ax.cal_state ?? 0;
        if (cal < maxCal) maxCal = cal;
        
        ['cine_', 'side_'].forEach(prefix => {
            const labelEl = document.getElementById(`${prefix}${axis}_cal`);
            if (labelEl) {
                labelEl.innerText = CAL_LABELS[cal] || '';
                labelEl.style.color = SPEED_COLORS[cal];
            }
        });
    });

    ['cine_', 'side_'].forEach(prefix => {
        const badge = document.getElementById(`${prefix}speed_badge`);
        if (badge) {
            badge.innerText = SPEED_LABELS[maxCal] || 'CRAWL ONLY';
            badge.style.color = SPEED_COLORS[maxCal];
            badge.style.borderColor = SPEED_COLORS[maxCal];
        }
    });
}

function _updateLimitBars(posS, posP, posT) {
    // Update position indicators on limit bars
    function _setBar(barId, posId, pos, min, max) {
        const bar = document.getElementById(barId);
        const dot = document.getElementById(posId);
        if (!bar || !dot || min == null || max == null || max <= min) return;
        const pct = Math.max(0, Math.min(100, (pos - min) / (max - min) * 100));
        dot.style.left = pct + '%';
    }
    const sl = _cineLimits.slider || {};
    const pa = _cineLimits.pan || {};
    const ti = _cineLimits.tilt || {};
    
    ['cine_', 'side_'].forEach(prefix => {
        _setBar(`${prefix}slider_bar`, `${prefix}slider_pos`, posS, sl.min, sl.max);
        _setBar(`${prefix}pan_bar`,    `${prefix}pan_pos`,    posP, pa.min, pa.max);
        _setBar(`${prefix}tilt_bar`,   `${prefix}tilt_pos`,   posT, ti.min, ti.max);
    });
}

// ── Arctan tracker ───────────────────────────────────────────────────────────
function arctanMarkPoint() {
    sendCmd('arctan_add_point');
}

function arctanClear() {
    sendCmd('arctan_clear');
    const cb = document.getElementById('arctan_enable_cb');
    if (cb) { cb.checked = false; cb.disabled = true; }
    const badge = document.getElementById('arctan_lock_badge');
    if (badge) { badge.innerText = 'OFF'; badge.style.color = 'var(--text-muted)'; }
    document.getElementById('arctan_status_row').style.display = 'none';
    document.getElementById('arctan_point_count').innerText = '(0)';
}

function arctanEnable(enabled) {
    sendCmd('arctan_enable', { enabled });
}

function handleArctanStatus(data) {
    const countEl = document.getElementById('arctan_point_count');
    if (countEl) countEl.innerText = `(${data.points})`;

    const statusRow = document.getElementById('arctan_status_row');
    const cb = document.getElementById('arctan_enable_cb');
    const badge = document.getElementById('arctan_lock_badge');

    if (data.solved) {
        if (statusRow) statusRow.style.display = '';
        const res = document.getElementById('arctan_residual');
        if (res) res.innerText = (data.residual || 0).toFixed(2);
        const warn = document.getElementById('arctan_warning');
        if (warn) warn.innerText = data.warning || '';
        if (cb) cb.disabled = false;
    } else {
        if (statusRow) statusRow.style.display = 'none';
        if (cb) cb.disabled = true;
    }
}

function handleArctanEnabled(data) {
    const badge = document.getElementById('arctan_lock_badge');
    const cb = document.getElementById('arctan_enable_cb');
    if (badge) {
        badge.innerText = data.enabled ? 'LOCKED' : 'OFF';
        badge.style.color = data.enabled ? 'var(--accent-teal)' : 'var(--text-muted)';
        badge.style.borderColor = data.enabled ? 'var(--accent-teal)' : '#333';
    }
    if (cb) cb.checked = !!data.enabled;
}

// ── Inertia / live mode ──────────────────────────────────────────────────────
function updateInertiaLabel(param) {
    const el = document.getElementById(`cine_${param}`);
    const lbl = document.getElementById(`cine_${param}_val`);
    if (!el || !lbl) return;
    lbl.innerText = param === 'mass' ? el.value + 's' : el.value;
}

function sendInertia() {
    sendCmd('cinematic_set_inertia', {
        mass: parseFloat(document.getElementById('cine_mass')?.value || 0.4),
        drag: parseFloat(document.getElementById('cine_drag')?.value || 0.55),
    });
}

// ── Pan/tilt sensitivity — logarithmic scale ──────────────────────────────────
// The slider is a 0–100 integer but maps logarithmically to 0.025%–100% scale.
// Log scale is essential: a linear slider can't usefully span a 4000:1 range.
// Each drag increment feels equal across the whole range — fine tuning at the
// low end (telephoto) is as easy as coarse adjustment at the high end.
const PT_SCALE_MIN = 0.00025;   // 0.025% — floor for high-power telephoto lenses
const PT_SCALE_MAX = 1.0;       // 100%

function sliderToPtScale(v) {
    // Slider 0 → PT_SCALE_MIN,  slider 100 → PT_SCALE_MAX (log interpolation)
    if (v <= 0)   return PT_SCALE_MIN;
    if (v >= 100) return PT_SCALE_MAX;
    return PT_SCALE_MIN * Math.pow(PT_SCALE_MAX / PT_SCALE_MIN, v / 100);
}

function ptScaleToSlider(scale) {
    // Inverse: actual scale value → slider position 0–100
    if (scale <= PT_SCALE_MIN) return 0;
    if (scale >= PT_SCALE_MAX) return 100;
    return Math.round(100 * Math.log(scale / PT_SCALE_MIN)
                              / Math.log(PT_SCALE_MAX / PT_SCALE_MIN));
}

function updatePtScaleLabel() {
    const el  = document.getElementById('cine_pt_scale');
    const lbl = document.getElementById('cine_pt_scale_val');
    if (!el || !lbl) return;
    const scale = sliderToPtScale(parseInt(el.value));
    const pct   = scale * 100;
    // Show as ×multiplier for values ≥ 10%, as percentage below that —
    // "×1.00" and "×0.15" are immediately readable as speed ratios, while
    // "1.5%" and "0.03%" convey ultra-fine precision at the low end.
    if (pct >= 10) {
        lbl.innerText = '×' + scale.toFixed(2);
    } else if (pct >= 1) {
        lbl.innerText = pct.toFixed(1) + '%';
    } else if (pct >= 0.1) {
        lbl.innerText = pct.toFixed(2) + '%';
    } else {
        lbl.innerText = pct.toFixed(3) + '%';
    }
    // Colour-code: teal = normal/full, gold = reduced, red = ultra-fine
    lbl.style.color = scale >= 0.50 ? 'var(--accent-teal)'
                    : scale >= 0.05 ? 'var(--accent-gold)'
                    :                 'var(--accent-red)';
}

function sendPtSensitivity() {
    const el = document.getElementById('cine_pt_scale');
    if (!el) return;
    const scale = sliderToPtScale(parseInt(el.value));
    sendCmd('cinematic_set_pt_sensitivity', { scale });
}

function setRigPreset(name) {
    const presets = {
        light:    { mass: 0.15, drag: 0.80, ptScale: 100 },   // 100% sensitivity
        standard: { mass: 0.40, drag: 0.55, ptScale: 100 },
        heavy:    { mass: 0.90, drag: 0.30, ptScale: 100 },
        // Tracking: near-zero inertia + ~15% pan/tilt speed for telephoto precision.
        // ptScale is slider position (0–100 log scale); 77 ≈ 15% actual scale.
        // User can fine-tune live with the slider without losing the mass/drag.
        tracking: { mass: 0.02, drag: 0.20, ptScale: 77  },
    };
    const p = presets[name];
    if (!p) return;
    document.getElementById('cine_mass').value = p.mass;
    document.getElementById('cine_drag').value = p.drag;
    updateInertiaLabel('mass');
    updateInertiaLabel('drag');
    // Sync the pan/tilt sensitivity slider to this preset's default
    const ptEl = document.getElementById('cine_pt_scale');
    if (ptEl) { ptEl.value = p.ptScale; updatePtScaleLabel(); }
    sendCmd('cinematic_set_inertia', { preset: name });
    // Update preset button highlight (including Tracking)
    ['light', 'standard', 'heavy', 'tracking'].forEach(n => {
        const btn = document.getElementById(
            `preset${n.charAt(0).toUpperCase() + n.slice(1)}`);
        if (btn) btn.classList.toggle('active', n === name);
    });
    log(`Preset: ${name}${name === 'tracking' ? ' — pan/tilt 15% speed, direct response' : ''}`);
}

function cinematicLiveStart() {
    if (isRunning) return;
    sendCmd('cinematic_live_start', {
        mass: parseFloat(document.getElementById('cine_mass')?.value || 0.4),
        drag: parseFloat(document.getElementById('cine_drag')?.value || 0.55),
    });
    _cineInertiaRunning = true;
    const btn = document.getElementById('liveStartBtn');
    if (btn) {
        btn.innerText = '■ Stop Live Control';
        btn.onclick = cinematicLiveStop;
    }
}

function cinematicLiveStop() {
    sendCmd('cinematic_live_stop');
    _cineInertiaRunning = false;
    const btn = document.getElementById('liveStartBtn');
    if (btn) {
        btn.innerText = '▶ Start Live Control';
        btn.onclick = cinematicLiveStart;
    }
}

// ── Keyframes ────────────────────────────────────────────────────────────────
function addKeyframeAtCurrent() {
    sendCmd('cinematic_add_keyframe', {
        duration_s: 3.0,
        easing: 'gaussian',
    });
}

function handleCineKeyframes(data) {
    _cineKeyframes = data.keyframes || [];
    _renderKeyframeList();
    _updatePathSummary();
    const btn = document.getElementById('reverse_move_btn');
    if (btn) {
        const rev = !!data.reversed;
        btn.style.color       = rev ? 'var(--accent-yellow, #f0c040)' : '';
        btn.style.borderColor = rev ? 'var(--accent-yellow, #f0c040)' : '';
        btn.title = rev
            ? '⇄ Move is REVERSED — clip will be recorded backward for reverse playback'
            : '⇄ Reverse move direction — record clip moving backward so reverse playback matches the plate';
    }
}

// ── Motion Path summary (shared across all modes) ─────────────────────────────
// Updates the path badge, path summary line, and mode-specific stats block.
// Called whenever keyframes change or the active mode/frame-count changes.
function _updatePathSummary() {
    const kfs = _cineKeyframes || [];
    const badge    = document.getElementById('path_kf_badge');
    const summary  = document.getElementById('path_summary');
    const tlCard      = document.getElementById('tl_motion_card');
    const tlBlock     = document.getElementById('tl_path_stats');
    const tlText      = document.getElementById('tl_stats_text');
    const tlReturnRow = document.getElementById('tl_return_row');
    const cnBlock  = document.getElementById('cine_path_stats');
    const cnText   = document.getElementById('cine_stats_text');
    const scalePan = document.getElementById('scaleDurPanel');
    const isTl     = (currentMode === 'timelapse');
    const isCn     = (currentMode === 'cinematic');

    // Scale Duration panel is cinematic-only — hide in timelapse to avoid confusion
    if (scalePan) scalePan.style.display = isCn ? '' : 'none';

    if (kfs.length === 0) {
        if (badge)        badge.innerText        = '0 keyframes';
        if (summary)      summary.innerText      = 'No path — jog to position and add keyframes';
        if (tlBlock)      tlBlock.style.display  = 'none';
        if (cnBlock)      cnBlock.style.display  = 'none';
        if (tlReturnRow)  tlReturnRow.style.display = 'none';

        // Timelapse: show a "no motion" card so the user knows what this section does
        if (tlCard) {
            if (isTl) {
                tlCard.style.display = '';
                tlCard.style.background = 'rgba(255,255,255,0.04)';
                tlCard.style.border = '1px solid #333';
                tlCard.innerHTML = `
                    <div style="font-size:0.72rem; color:var(--text-muted); line-height:1.6;">
                        <span style="color:var(--text-dim); font-weight:600;">⬜ No motion path</span>
                        — timelapse will shoot from a fixed position.<br>
                        Add keyframes below (or load a saved move) to enable motion.
                        The path auto-scales to your frame count — no timing setup needed.
                    </div>`;
            } else {
                tlCard.style.display = 'none';
            }
        }
        return;
    }

    if (badge) badge.innerText = `${kfs.length} keyframe${kfs.length !== 1 ? 's' : ''}`;

    // Axis ranges across all keyframes
    const sVals  = kfs.map(k => k.slider_mm);
    const pVals  = kfs.map(k => k.pan_deg);
    const tVals  = kfs.map(k => k.tilt_deg);
    const sRange = Math.max(...sVals) - Math.min(...sVals);
    const pRange = Math.max(...pVals) - Math.min(...pVals);
    const tRange = Math.max(...tVals) - Math.min(...tVals);

    const parts = [];
    if (sRange > 0.5) parts.push(`${sRange.toFixed(0)} mm slider`);
    if (pRange > 0.5) parts.push(`${pRange.toFixed(1)}° pan`);
    if (tRange > 0.5) parts.push(`${tRange.toFixed(1)}° tilt`);
    if (summary) summary.innerText = parts.length > 0
        ? parts.join(' · ')
        : 'Stationary (all keyframes at same position)';

    // Per-segment durations (used for proportional frame allocation)
    const segDurs    = kfs.slice(0, -1).map(k => Math.max(0.1, k.duration_s || 3.0));
    const totalSegDur = segDurs.reduce((a, b) => a + b, 0);

    // ── Timelapse motion card ──────────────────────────────────────────────────
    if (tlCard) {
        tlCard.style.display = isTl ? '' : 'none';
        if (isTl && kfs.length >= 2) {
            const frames   = parseInt(document.getElementById('total_frames')?.value || '300', 10) || 300;
            const interval = parseFloat(document.getElementById('manual_interval')?.value || '5') || 5;
            const segs     = kfs.length - 1;

            // Per-segment frame allocation (proportional to duration_s weights)
            const segFrames = segDurs.map(d => Math.round(d / totalSegDur * frames));

            // Per-frame movement distances
            const mmPerFrame  = sRange > 0 ? sRange / frames : 0;
            const panPerFrame = pRange > 0 ? pRange / frames : 0;
            const tiltPerFrame= tRange > 0 ? tRange / frames : 0;

            // Timing sanity: motors need ~1–2 s between shots at slow speeds.
            // Rough estimate: 1.5 s overhead always + ~0.003 s per mm + ~0.01 s per degree.
            const estMoveTime = 1.5 + mmPerFrame * 0.003 + (panPerFrame + tiltPerFrame) * 0.01;
            const timeOk = interval >= estMoveTime + 0.5;   // leave 0.5 s margin

            // Overrun warnings
            const warnings = [];
            if (sRange / frames > 120) warnings.push('slider step exceeds motor limit — add more frames or shorten path');
            if (pRange / frames > 37)  warnings.push('pan step exceeds motor limit');
            if (tRange / frames > 30)  warnings.push('tilt step exceeds motor limit');
            if (!timeOk) warnings.push(`interval (${interval}s) may be too short for motor moves — try ≥ ${Math.ceil(estMoveTime + 1)}s`);

            const statusColor = warnings.length ? 'var(--accent-gold)' : 'var(--accent-green,#3a7)';
            const statusIcon  = warnings.length ? '⚠' : '✅';
            const statusLabel = warnings.length ? 'Motion active — check warnings' : 'Motion active';

            let html = `
                <div style="display:flex; align-items:center; gap:8px; margin-bottom:6px;">
                    <span style="font-size:0.85rem;">${statusIcon}</span>
                    <span style="font-weight:700; font-size:0.78rem; color:${statusColor};">${statusLabel}</span>
                    <span style="font-size:0.68rem; color:var(--text-muted); margin-left:auto;">
                        ${frames} frames · ${segs} seg${segs !== 1 ? 's' : ''}
                    </span>
                </div>
                <div style="font-size:0.7rem; color:var(--text-dim); font-family:monospace;
                            line-height:1.7; border-top:1px solid #333; padding-top:6px;">`;

            if (mmPerFrame  > 0) html += `Slider  ${mmPerFrame.toFixed(2)} mm/frame<br>`;
            if (panPerFrame > 0) html += `Pan     ${panPerFrame.toFixed(3)}°/frame<br>`;
            if (tiltPerFrame> 0) html += `Tilt    ${tiltPerFrame.toFixed(3)}°/frame<br>`;
            if (segs > 1) {
                html += `Seg frames  ${segFrames.join(' · ')}<br>`;
            }
            html += `Est. move time  ~${estMoveTime.toFixed(1)}s  (interval ${interval}s)`;
            html += `</div>`;

            if (warnings.length) {
                html += `<div style="margin-top:6px; font-size:0.68rem; color:var(--accent-gold);
                                     line-height:1.5;">`;
                warnings.forEach(w => { html += `⚠ ${w}<br>`; });
                html += `</div>`;
            }

            tlCard.style.background = warnings.length
                ? 'rgba(255,180,0,0.07)'
                : 'rgba(34,170,100,0.09)';
            tlCard.style.border = `1px solid ${warnings.length ? 'rgba(255,180,0,0.3)' : 'rgba(34,170,100,0.3)'}`;
            tlCard.innerHTML = html;
        } else if (isTl && kfs.length === 1) {
            tlCard.style.background = 'rgba(255,255,255,0.04)';
            tlCard.style.border = '1px solid #333';
            tlCard.innerHTML = `<div style="font-size:0.72rem; color:var(--text-muted);">
                ⬜ Need at least 2 keyframes for motion. Add a second point to activate.</div>`;
        }
    }

    // ── Return to Start button (timelapse only, needs ≥2 keyframes) ──────────
    if (tlReturnRow) tlReturnRow.style.display = (isTl && kfs.length >= 2) ? '' : 'none';

    // ── Timelapse detail block (secondary, shown below card) ──────────────────
    if (tlBlock) {
        const show = isTl && kfs.length >= 2;
        tlBlock.style.display = show ? '' : 'none';
        if (show && tlText) {
            const frames = parseInt(document.getElementById('total_frames')?.value || '300', 10) || 300;
            const segs   = kfs.length - 1;
            const segFrames = segDurs.map(d => Math.round(d / totalSegDur * frames));
            const lines = [
                `Path: ${_pathMode === 'catmull_rom' ? 'Catmull-Rom' : 'Linear'} · Easing: ${_globalEasing}`,
            ];
            if (segs > 1) lines.push(`Frame distribution: ${segFrames.join(' → ')}`);
            tlText.innerHTML = lines.join('<br>');
        }
    }

    // ── Cinematic stats ────────────────────────────────────────────────────────
    if (cnBlock) {
        cnBlock.style.display = (isCn && kfs.length >= 2) ? '' : 'none';
        if (isCn && kfs.length >= 2) {
            const segs = kfs.length - 1;
            const pathLabel = _pathMode === 'catmull_rom'
                ? `Catmull-Rom (tension ${_catmullTension.toFixed(2)})`
                : 'Linear';
            const lines = [
                `${totalSegDur.toFixed(1)}s total · ${segs} segment${segs !== 1 ? 's' : ''}`,
                `Path: ${pathLabel} · Easing: ${_globalEasing}`,
            ];
            if (sRange > 0.5) lines.push(`Slider: avg ${(sRange / totalSegDur).toFixed(1)} mm/s`);
            if (pRange > 0.5) lines.push(`Pan:    avg ${(pRange / totalSegDur).toFixed(2)}°/s`);
            if (tRange > 0.5) lines.push(`Tilt:   avg ${(tRange / totalSegDur).toFixed(2)}°/s`);
            if (_pathMinDuration > 0 && totalSegDur < _pathMinDuration) {
                lines.push(`⚠ Move too fast — min safe duration: ${_pathMinDuration.toFixed(1)}s`
                    + (_pathMinAxis ? ` (${_pathMinAxis})` : ''));
            }
            if (cnText) cnText.innerHTML = lines.join('<br>');
        }
    }
}

function _renderKeyframeList() {
    const container = document.getElementById('cine_kf_list');
    if (!container) return;

    const kfs = _cineKeyframes;

    // ── Scale panel visibility ────────────────────────────────────────────────
    const scalePanel = document.getElementById('scaleDurPanel');
    const scaleDurInput = document.getElementById('scaleDurInput');
    const currentTotalEl = document.getElementById('currentTotalDur');

    if (kfs.length < 2) {
        container.innerHTML = `<div style="color:var(--text-muted); font-size:0.72rem;
            text-align:center; padding:12px 0;">No keyframes. Jog to position and click Add.</div>`;
        if (scalePanel) scalePanel.style.display = 'none';
        return;
    }

    // Total duration from all segments (all keyframes except last)
    const totalDur = kfs.slice(0, -1).reduce((a, k) => a + Math.max(0.1, k.duration_s || 3.0), 0);

    // Scale Duration panel is cinematic-only — never show in timelapse
    if (scalePanel) scalePanel.style.display = (currentMode === 'cinematic') ? '' : 'none';
    if (currentTotalEl) currentTotalEl.textContent = `${totalDur.toFixed(1)}s`;
    // Pre-fill scale input with current total if user hasn't changed it
    if (scaleDurInput && scaleDurInput.dataset.userSet !== 'true') {
        scaleDurInput.value = Math.round(totalDur);
    }

    // Full list of supported easing curves (matches distributions.py CURVE_FUNCTIONS keys)
    const EASINGS = [
        'cycloid', 'gaussian', 'parabolic', 'inverted_parabolic',
        'even', 'linear', 'catenary', 'inverted_catenary',
        'ellipsoidal', 'inverted_ellipsoidal', 'lame', 'inverted_lame',
    ];

    const nSegs = kfs.length - 1;   // number of segments

    container.innerHTML = kfs.map((kf, i) => {
        const isLastKf  = (i === kfs.length - 1);   // final endpoint — no outgoing segment
        const isLastSeg = (i === kfs.length - 2);   // last segment — auto-fills (read-only %)
        const easingIsGlobal = (kf.easing === _globalEasing);
        const easingLabel = easingIsGlobal
            ? `<span style="font-size:0.6rem; color:var(--text-muted);">(global)</span>`
            : `<span style="font-size:0.6rem; color:var(--accent-gold);">override</span>`;

        let segRow = '';

        if (isLastKf) {
            // Final keyframe — just the position label
            segRow = `<div style="color:var(--text-muted); font-size:0.68rem; padding-top:2px;">
                          End point</div>`;

        } else {
            const dur  = Math.max(0.1, kf.duration_s || 3.0);
            const pct  = totalDur > 0 ? (dur / totalDur * 100) : (100 / nSegs);
            const pctDisplay = pct.toFixed(1);
            const secsLabel  = dur.toFixed(1);

            // Proportion bar
            const bar = `<div style="height:3px; background:#1a1a1a; border-radius:2px; margin-bottom:5px;">
                <div style="height:100%; width:${Math.min(100, pct).toFixed(1)}%;
                            background:var(--accent-teal); border-radius:2px; opacity:0.55;"></div>
            </div>`;

            // Easing select (same for editable and auto segments)
            const easingSelect = `
                <div style="display:flex; flex-direction:column; flex:1; gap:2px; min-width:0;">
                    <select style="font-size:0.7rem; width:100%;"
                            onchange="sendCmd('cinematic_update_keyframe',
                                      {index:${i}, easing:this.value})">
                        ${EASINGS.map(e =>
                            `<option value="${e}" ${e === kf.easing ? 'selected' : ''}>${e}</option>`
                        ).join('')}
                    </select>
                    <div style="text-align:right;">${easingLabel}</div>
                </div>`;

            if (isLastSeg) {
                // Last segment: percentage auto-fills, read-only
                segRow = `
                <div>
                    ${bar}
                    <div style="display:flex; gap:6px; align-items:center;">
                        <span style="font-size:0.68rem; color:var(--text-muted);
                                     white-space:nowrap;">→ remaining</span>
                        <span style="font-size:0.8rem; color:var(--text-dim);
                                     font-family:monospace; font-weight:600;
                                     background:#1a1a1a; border:1px solid #333;
                                     border-radius:4px; padding:2px 7px;">${pctDisplay}%</span>
                        <span style="font-size:0.65rem; color:var(--text-muted);
                                     font-family:monospace;">${secsLabel}s</span>
                        ${easingSelect}
                    </div>
                </div>`;
            } else {
                // Editable segment: % input, next segment absorbs change
                segRow = `
                <div>
                    ${bar}
                    <div style="display:flex; gap:6px; align-items:center;">
                        <span style="font-size:0.68rem; color:var(--text-muted);
                                     white-space:nowrap;">→</span>
                        <input type="number"
                               value="${pctDisplay}"
                               min="1" max="99" step="1"
                               style="width:52px; font-size:0.8rem; padding:2px 5px;
                                      font-family:monospace; text-align:right;"
                               onchange="sendCmd('cinematic_set_segment_pct',
                                         {index:${i}, pct:parseFloat(this.value)})">
                        <span style="font-size:0.7rem; color:var(--text-muted);">%</span>
                        <span style="font-size:0.65rem; color:var(--text-muted);
                                     font-family:monospace; min-width:32px;">${secsLabel}s</span>
                        ${easingSelect}
                    </div>
                </div>`;
            }
        }

        return `
        <div style="border-bottom:1px solid #222; padding:6px 8px; font-size:0.7rem;">
            <div style="display:flex; justify-content:space-between; align-items:center;
                        margin-bottom:4px;">
                <span style="color:var(--accent-teal); font-weight:600;">KF ${i + 1}</span>
                <span style="color:var(--text-dim); font-family:monospace; font-size:0.67rem;">
                    s:${kf.slider_mm.toFixed(1)}  p:${kf.pan_deg.toFixed(1)}°  t:${kf.tilt_deg.toFixed(1)}°
                </span>
                <button onclick="sendCmd('cinematic_remove_keyframe',{index:${i}})"
                        style="background:none; border:1px solid #440000; color:#cc2200;
                               border-radius:3px; padding:1px 6px; cursor:pointer; font-size:0.65rem;">✕</button>
            </div>
            ${segRow}
        </div>`;
    }).join('');
}

// ── Playback ─────────────────────────────────────────────────────────────────
function cinematicPlay() {
    if (isRunning) { log('Already running.'); return; }
    if (_cineKeyframes.length < 2) {
        log('⚠ Need at least 2 keyframes to play a move.');
        return;
    }
    // Origin check removed — hardware zero sets the implicit origin (0,0,0).
    // Server-side preflight handles any actual limit/path violations.
    sendCmd('cinematic_play');
}

let _returnToStartBusy = false;
function returnToStart() {
    if (_returnToStartBusy || isRunning) {
        log('⚠ Movement already in progress — wait for rig to stop.');
        return;
    }
    _returnToStartBusy = true;
    // Disable all Return to Start buttons while moving
    document.querySelectorAll('#returnToStartBtn, [onclick="returnToStart()"]').forEach(b => {
        b.disabled = true;
        b.style.opacity = '0.5';
    });
    sendCmd('cinematic_return_to_start');
    log('↩ Returning to first keyframe position…');
    // Re-enable after move completes (server broadcasts run_state: running→false)
    // Fallback: re-enable after 30s in case WS event is missed
    setTimeout(() => _clearReturnToStartBusy(), 30000);
}

function _clearReturnToStartBusy() {
    _returnToStartBusy = false;
    document.querySelectorAll('#returnToStartBtn, [onclick="returnToStart()"]').forEach(b => {
        b.disabled = false;
        b.style.opacity = '';
    });
}

function handleCineProgress(data) {
    const wrap = document.getElementById('cine_prog_bar_wrap');
    const bar = document.getElementById('cine_prog_bar');
    const msg = document.getElementById('cine_prog_msg');
    if (wrap) wrap.style.display = '';
    if (msg) msg.style.display = '';
    if (bar) bar.style.width = Math.round((data.progress || 0) * 100) + '%';
    const pct = Math.round((data.progress || 0) * 100);
    if (msg) msg.innerText = `${pct}% — `
        + `s:${data.pos_s?.toFixed(1)}mm p:${data.pos_p?.toFixed(1)}° t:${data.pos_t?.toFixed(1)}°`;
}

function handleCinePlayDone() {
    const wrap = document.getElementById('cine_prog_bar_wrap');
    const msg = document.getElementById('cine_prog_msg');
    if (wrap) wrap.style.display = 'none';
    if (msg) msg.style.display = 'none';
}

// ── Origin ───────────────────────────────────────────────────────────────────
function handleCineOriginSet(data) {
    _cineOrigin = data;
    const isSet = data && (data.slider_mm !== 0 || data.pan_deg !== 0 || data.tilt_deg !== 0);

    // Origin display inside Advanced panel
    const el = document.getElementById('cine_origin_disp');
    if (el) {
        if (!isSet) {
            el.innerText = 'not set';
            el.style.color = '';
        } else {
            el.innerText = `s:${data.slider_mm?.toFixed(1)}mm `
                + `p:${data.pan_deg?.toFixed(1)}° t:${data.tilt_deg?.toFixed(1)}°`;
            el.style.color = 'var(--accent-teal)';
        }
    }

    // Clear button inside Advanced panel
    const clearBtn = document.getElementById('clearOriginBtn');
    if (clearBtn) clearBtn.style.display = isSet ? '' : 'none';

    // Warning banner above Return to Start
    const banner = document.getElementById('originWarnBanner');
    if (banner) banner.style.display = isSet ? 'block' : 'none';
}

// ── Reference point ───────────────────────────────────────────────────────────
function handleCineRefSaved(data) {
    const el  = document.getElementById('cine_ref_disp');
    const row = document.getElementById('cine_at_ref_row');
    if (el) {
        // Show design-space offset from start, or physical position if provided
        if (data.phys_slider_mm !== null && data.phys_slider_mm !== undefined) {
            el.innerText = `s:${data.phys_slider_mm?.toFixed(1)}mm `
                + `p:${data.phys_pan_deg?.toFixed(1)}° `
                + `t:${data.phys_tilt_deg?.toFixed(1)}°`;
        } else {
            // Loaded from library — show design-space offset
            const sign = v => v >= 0 ? `+${v.toFixed(1)}` : `${v.toFixed(1)}`;
            el.innerText = `Δs:${sign(data.slider_mm)}mm `
                + `Δp:${sign(data.pan_deg)}° `
                + `Δt:${sign(data.tilt_deg)}°`;
        }
        el.style.color = 'var(--accent)';
    }
    // Show the "At Reference" button
    if (row) row.style.display = '';
}

function handleCineRefCleared() {
    const el  = document.getElementById('cine_ref_disp');
    const row = document.getElementById('cine_at_ref_row');
    if (el)  { el.innerText = 'not saved'; el.style.color = ''; }
    if (row) row.style.display = 'none';
}

// ── Move library ─────────────────────────────────────────────────────────────
function saveCurrentMove() {
    const name = document.getElementById('cine_move_name')?.value?.trim();
    if (!name) { log('Enter a move name first.'); return; }
    sendCmd('cinematic_save_move', { name });
    log(`Saving move '${name}'…`);
}

function handleCineMoves(data) {
    const moves = data.moves || [];
    const container = document.getElementById('cine_move_library');
    if (!container) return;

    if (moves.length === 0) {
        container.innerHTML = `<div style="color:var(--text-muted); font-size:0.72rem;
            text-align:center; padding:12px;">No saved moves.</div>`;
        return;
    }

    container.innerHTML = moves.map(m => `
        <div style="display:flex; gap:6px; align-items:center; padding:6px 8px;
                    border-bottom:1px solid #1a1a1a;">
            <div style="flex:1; min-width:0;">
                <div style="font-size:0.78rem; color:var(--accent-teal);
                            white-space:nowrap; overflow:hidden; text-overflow:ellipsis;">
                    ${m.name}</div>
                <div style="font-size:0.62rem; color:var(--text-muted);">
                    ${m.keyframes} kf · ${m.duration?.toFixed(1)}s ·
                    ${new Date(m.created).toLocaleDateString()}</div>
            </div>
            <button onclick="sendCmd('cinematic_load_move',{name:'${m.name}'})"
                    style="font-size:0.65rem; padding:3px 8px; background:none;
                           border:1px solid var(--accent-teal); color:var(--accent-teal);
                           border-radius:4px; cursor:pointer; white-space:nowrap;">Load</button>
            <button onclick="if(confirm('Delete \\'${m.name}\\'?')) sendCmd('cinematic_delete_move',{name:'${m.name}'})"
                    style="font-size:0.65rem; padding:3px 8px; background:none;
                           border:1px solid #440000; color:#cc2200;
                           border-radius:4px; cursor:pointer;">✕</button>
        </div>
    `).join('');
}

// ── Recording ────────────────────────────────────────────────────────────────
function toggleRecord() {
    if (_cineRecording) {
        sendCmd('record_stop');
    } else {
        sendCmd('record_start');
    }
}

function handleRecordState(data) {
    _cineRecording = data.recording;
    const btn = document.getElementById('recordBtn');
    const ind = document.getElementById('recIndicator');
    const tc = document.getElementById('timecodeDisplay');

    if (data.recording) {
        _cineRecordStart = Date.now();
        if (btn) {
            btn.innerText = '■ STOP REC';
            btn.style.background = '#2a0000';
            btn.style.borderColor = '#ff2200';
            btn.style.color = '#ff3300';
        }
        if (ind) { ind.style.background = '#ff2200'; ind.style.boxShadow = '0 0 8px #ff2200'; }
        // Start timecode ticker
        if (_cineRecordTimer) clearInterval(_cineRecordTimer);
        _cineRecordTimer = setInterval(() => {
            const elapsed = (Date.now() - _cineRecordStart) / 1000;
            const h = Math.floor(elapsed / 3600);
            const m = Math.floor((elapsed % 3600) / 60);
            const s = Math.floor(elapsed % 60);
            const f = Math.floor((elapsed % 1) * 30);
            if (tc) tc.innerText =
                `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:` +
                `${String(s).padStart(2, '0')}:${String(f).padStart(2, '0')}`;
        }, 33);
    } else {
        if (_cineRecordTimer) { clearInterval(_cineRecordTimer); _cineRecordTimer = null; }
        if (btn) {
            btn.innerText = '● REC';
            btn.style.background = '#1a0000';
            btn.style.borderColor = '#cc2200';
            btn.style.color = '#ff4433';
        }
        if (ind) { ind.style.background = '#333'; ind.style.boxShadow = 'none'; }
        if (tc) tc.innerText = '00:00:00:00';
    }
}

// ── Full state restore ────────────────────────────────────────────────────────
function handleCineState(data) {
    if (data.limits) handleCineLimits({ limits: data.limits });
    if (data.keyframes) handleCineKeyframes({ keyframes: data.keyframes });
    if (data.moves) handleCineMoves({ moves: data.moves });
    if (data.arctan) {
        const ct = document.getElementById('arctan_point_count');
        if (ct) ct.innerText = `(${data.arctan.points})`;
        if (data.arctan.solved) {
            document.getElementById('arctan_status_row').style.display = '';
            const res = document.getElementById('arctan_residual');
            if (res) res.innerText = (data.arctan.residual || 0).toFixed(2);
            const cb = document.getElementById('arctan_enable_cb');
            if (cb) cb.disabled = false;
        }
    }
    if (data.inertia) {
        const massEl  = document.getElementById('cine_mass');
        const dragEl  = document.getElementById('cine_drag');
        const scaleEl = document.getElementById('cine_pt_scale');
        if (massEl)  { massEl.value  = data.inertia.mass;  updateInertiaLabel('mass'); }
        if (dragEl)  { dragEl.value  = data.inertia.drag;  updateInertiaLabel('drag'); }
        if (scaleEl && data.inertia.pan_tilt_scale !== undefined) {
            scaleEl.value = ptScaleToSlider(data.inertia.pan_tilt_scale);
            updatePtScaleLabel();
        }
    }
    if (data.recording) handleRecordState({ recording: data.recording });
    if (data.rail_tilt !== undefined) {
        const el = document.getElementById('rail_tilt_deg');
        if (el) el.value = data.rail_tilt;
    }
    if (data.high_power !== undefined) {
        const el = document.getElementById('high_power_mode');
        if (el) el.checked = data.high_power;
    }
    if (data.origin && data.origin.slider_mm !== undefined) {
        handleCineOriginSet(data.origin);
    }
    if (data.reference && data.reference.slider_mm !== undefined) {
        handleCineRefSaved(data.reference);
    } else if (data.reference !== undefined) {
        handleCineRefCleared();
    }
    // Path planning fields
    if (data.path_mode) {
        _pathMode = data.path_mode;
        _applyPathMode(data.path_mode);
    }
    if (data.global_easing) {
        _globalEasing = data.global_easing;
        _applyGlobalEasing(data.global_easing);
    }
    if (data.catmull_tension !== undefined) {
        _catmullTension = data.catmull_tension;
        _applyCatmullTension(data.catmull_tension);
    }
    if (data.min_duration !== undefined) {
        _pathMinDuration = data.min_duration;
        _pathMinAxis     = data.min_duration_axis || '';
    }
    if (data.total_duration_s !== undefined && data.total_duration_s > 0) {
        const inp = document.getElementById('scaleDurInput');
        if (inp && inp.dataset.userSet !== 'true') {
            inp.value = Math.round(data.total_duration_s);
        }
    }
    _updatePathSummary();
}

// ── Path planning helpers ────────────────────────────────────────────────────

function _applyPathMode(mode) {
    const btnLinear  = document.getElementById('pathModeLinear');
    const btnCatmull = document.getElementById('pathModeCatmull');
    const tensionRow = document.getElementById('catmullTensionRow');
    const isCR = (mode === 'catmull_rom');
    if (btnLinear)  { btnLinear.style.background  = isCR ? 'none' : 'var(--accent-teal)';
                      btnLinear.style.color        = isCR ? '#555' : '#000';
                      btnLinear.style.borderColor  = isCR ? '#333' : 'var(--accent-teal)'; }
    if (btnCatmull) { btnCatmull.style.background = isCR ? 'var(--accent-teal)' : 'none';
                      btnCatmull.style.color       = isCR ? '#000' : '#555';
                      btnCatmull.style.borderColor = isCR ? 'var(--accent-teal)' : '#333'; }
    if (tensionRow) tensionRow.style.display = isCR ? '' : 'none';
}

function _applyGlobalEasing(curve) {
    const sel = document.getElementById('globalEasingSelect');
    if (sel) sel.value = curve;
}

function _applyCatmullTension(tension) {
    const slider = document.getElementById('catmullTensionSlider');
    const label  = document.getElementById('catmullTensionVal');
    if (slider) slider.value = tension;
    if (label)  label.textContent = parseFloat(tension).toFixed(2);
}

// ── Duration scaling ──────────────────────────────────────────────────────────

function scaleAllDurations() {
    const input = document.getElementById('scaleDurInput');
    if (!input) return;
    const newTotal = parseFloat(input.value);
    if (!newTotal || newTotal < 0.5) {
        log('⚠ Enter a target duration ≥ 0.5 s');
        return;
    }
    input.dataset.userSet = 'true';   // prevent auto-fill overwriting user's value
    sendCmd('cinematic_scale_duration', { total_s: newTotal });
}

function setPathMode(mode) {
    _applyPathMode(mode);
    sendCmd('cinematic_set_path_mode', { mode });
}

function setGlobalEasing(curve) {
    sendCmd('cinematic_set_global_easing', { curve });
    _renderKeyframeList();   // refresh segment labels
}

function setCatmullTension(value) {
    const t = parseFloat(value);
    _applyCatmullTension(t);
    sendCmd('cinematic_set_tension', { tension: t });
}

// ── Gamepad button events from server ────────────────────────────────────────
function handleGamepadBtn(data) {
    const btn = data.btn;
    if (btn === 'record') toggleRecord();
    if (btn === 'play') cinematicPlay();
    if (btn === 'stop') sendCmd('cinematic_stop');
    if (btn === 'arctan_toggle') {
        const cb = document.getElementById('arctan_enable_cb');
        if (cb && !cb.disabled) { cb.checked = !cb.checked; arctanEnable(cb.checked); }
    }
}

function handleGamepadStatus(data) {
    const connected = data.connected;
    log(connected ? '🎮 Controller connected.' : '⚠ Controller disconnected.');
    // Update persistent indicator in the control panel
    const ind = document.getElementById('gamepadIndicator');
    if (ind) {
        ind.textContent  = connected ? '🎮 Controller' : '🎮 No controller';
        ind.style.color  = connected ? 'var(--accent-green)' : 'var(--text-muted)';
    }
    if (!connected) _clearGamepadMirror();
}

// ── Gamepad input mirroring ───────────────────────────────────────────────────
// Reflects controller joystick positions and button presses onto the GUI so the
// user can see exactly what the controller is doing in real time.

function handleGamepadInput(data) {
    // ── Left stick → joystick pad knob ───────────────────────────────────────
    const knob = els.joystickKnob;
    const pad  = els.joystickPad;
    if (knob && pad && !joystickActive) {
        // Only move knob from gamepad when user isn't dragging it with a pointer
        const maxR = pad.getBoundingClientRect().width / 2;
        const dx   = (data.pan  || 0) * maxR;
        const dy   = -(data.tilt || 0) * maxR;   // tilt up = knob up = negative CSS y
        knob.style.left = `calc(50% + ${dx.toFixed(1)}px)`;
        knob.style.top  = `calc(50% + ${dy.toFixed(1)}px)`;
    }

    // ── Right stick → slider-strip knob ──────────────────────────────────────
    const sliderKnob  = document.getElementById('sliderStripKnob');
    const sliderTrack = document.getElementById('sliderStrip');
    if (sliderKnob && sliderTrack && !sliderStripActive) {
        // slider value is -1…+1; map to 0…100% strip position
        const frac = ((data.slider || 0) + 1) / 2;
        sliderKnob.style.left = `${(frac * 100).toFixed(1)}%`;
    }

    // ── D-pad → jog button highlights ────────────────────────────────────────
    // dpad_y convention (from Linux ABS_HAT0Y): -1 = physical UP, +1 = physical DOWN.
    // The motor command negates it so UP→positive tilt, DOWN→negative tilt.
    // The GUI must follow the same sign so the correct arrow lights up.
    const dx = data.dpad_x || 0;
    const dy = data.dpad_y || 0;
    _gpHighlight('jogPanLeft',   dx < -0.5);
    _gpHighlight('jogPanRight',  dx >  0.5);
    _gpHighlight('jogTiltUp',    dy < -0.5);   // dpad_y=-1 → physical UP → tilt UP
    _gpHighlight('jogTiltDown',  dy >  0.5);   // dpad_y=+1 → physical DOWN → tilt DOWN

    // ── L1 / R1 → slider nudge button highlights ─────────────────────────────
    const l1 = data.l1 || false;
    const r1 = data.r1 || false;
    _gpHighlight('jogSliderLeft',  l1);
    _gpHighlight('jogSliderRight', r1);
}

function _gpHighlight(id, on) {
    const el = document.getElementById(id);
    if (!el) return;
    if (on) {
        el.classList.add('gp-active');
        el.style.opacity = '';   // let CSS handle it when d-pad pressed
    } else {
        el.classList.remove('gp-active');
        el.style.opacity = '';
    }
}

function _clearGamepadMirror() {
    // Reset joystick knob to centre
    if (els.joystickKnob && !joystickActive) {
        els.joystickKnob.style.left = '50%';
        els.joystickKnob.style.top  = '50%';
    }
    // Reset slider knob to centre
    const sk = document.getElementById('sliderStripKnob');
    if (sk && !sliderStripActive) sk.style.left = '50%';
    // Clear all gamepad highlights
    ['jogPanLeft','jogPanRight','jogTiltUp','jogTiltDown',
     'jogSliderLeft','jogSliderRight'].forEach(id => _gpHighlight(id, false));
}


// ═══════════════════════════════════════════════════════════════════════════════
// PREVIEW CAMERA TOGGLE (Sony framing ↔ PiCam motion zone)
// ═══════════════════════════════════════════════════════════════════════════════

let _HAS_PICAM_FEED = true;  // false when Sony is the preview source (loupe polling pauses)

function togglePreviewCamera() {
    const btn = document.getElementById('previewToggleBtn');
    const cur = btn?.dataset.cam || 'picam';
    setPreviewCamera(cur === 'picam' ? 'sony' : 'picam');
}

function setPreviewCamera(cam) {
    const btn = document.getElementById('previewToggleBtn');
    if (btn) { btn.dataset.cam = cam; btn.innerText = cam === 'sony' ? '📷 Sony' : '📷 PiCam'; }
    sendCmd('set_preview_camera', { camera: cam });
    if (els.mjpegFeed) els.mjpegFeed.src = `/video_feed?t=${Date.now()}`;
    _HAS_PICAM_FEED = (cam === 'picam');
    log(`Preview: ${cam === 'sony' ? 'Sony (framing)' : 'PiCam (motion zone)'}`);
}

function updatePreviewToggleVisibility() {
    const cam = document.getElementById('camera_select')?.value;
    const btn = document.getElementById('previewToggleBtn');
    if (!btn) return;
    btn.style.display = (cam === 'sony') ? 'inline-block' : 'none';
    if (cam !== 'sony') setPreviewCamera('picam');
}


// ═══════════════════════════════════════════════════════════════════════════════
// SLIDER STRIP (linear rail axis — horizontal single-axis joystick)
// ═══════════════════════════════════════════════════════════════════════════════

function setupSliderStrip() {
    const track = document.getElementById('sliderStrip');
    const knob = document.getElementById('sliderStripKnob');
    if (!track || !knob) return;

    let dragging = false;
    let lastVel = 0;

    function getVelFromX(clientX) {
        const rect = track.getBoundingClientRect();
        const frac = Math.max(0, Math.min(1, (clientX - rect.left) / rect.width));
        const vel = (frac - 0.5) * 2;   // -1 to +1
        return Math.abs(vel) < 0.10 ? 0 : vel;   // ±10% dead zone
    }

    function updateKnob(clientX) {
        const rect = track.getBoundingClientRect();
        const frac = Math.max(0, Math.min(1, (clientX - rect.left) / rect.width));
        knob.style.left = `${frac * 100}%`;
    }

    let _lastSliderSendMs = 0;   // throttle: max 50 Hz, matching InertiaEngine tick rate
    function sendVel(vel) {
        const _now = Date.now();
        // Stop command (vel === 0) always sent immediately — never suppress it.
        // Movement sends throttled to 50 Hz: sending faster than the InertiaEngine
        // tick rate (20 ms) only queues stale commands that arrive after the next
        // physics step has already consumed a more recent value.
        if (vel !== 0 && _now - _lastSliderSendMs < 20) return;
        _lastSliderSendMs = _now;
        lastVel = vel;
        sliderVz = vel;    // keep shared state current so joystick pad includes it
        sendCmd('joystick', { vx: 0, vy: 0, vz: vel });
    }

    function stopSlider() {
        dragging = false;
        sliderStripActive = false;
        sliderVz = 0;
        track.classList.remove('active');
        knob.style.left = '50%';
        sendVel(0);
    }

    track.addEventListener('pointerdown', (e) => {
        dragging = true;
        sliderStripActive = true;
        track.classList.add('active');
        track.setPointerCapture(e.pointerId);
        updateKnob(e.clientX);
        sendVel(getVelFromX(e.clientX));
        e.preventDefault();
    });
    track.addEventListener('pointermove', (e) => { if (!dragging) return; updateKnob(e.clientX); sendVel(getVelFromX(e.clientX)); });
    track.addEventListener('pointerup', stopSlider);
    track.addEventListener('pointercancel', stopSlider);
}


// ═══════════════════════════════════════════════════════════════════════════════
// FOLDER BROWSER (complete implementation)
// ═══════════════════════════════════════════════════════════════════════════════

async function openFolderBrowser() {
    folderBrowserCurrentPath = document.getElementById('save_path')?.value || '/home/tim/Pictures';
    document.getElementById('folderModal').style.display = 'flex';
    cancelCreateFolder();
    await browseTo(folderBrowserCurrentPath);
}

function closeFolderBrowser(e) {
    if (!e || e.target === document.getElementById('folderModal')) {
        document.getElementById('folderModal').style.display = 'none';
        cancelCreateFolder();
    }
}

async function browseTo(path) {
    folderBrowserCurrentPath = path;
    document.getElementById('folderModalPath').innerText = path;
    document.getElementById('folderSelectedPath').innerText = path;
    cancelCreateFolder();

    // Highlight matching drive chip
    document.querySelectorAll('.drive-chip').forEach(c => c.classList.remove('active'));
    const chipMap = {
        'chip_home': '/home/tim',
        'chip_pics': '/home/tim/Pictures',
        'chip_media': '/media',
        'chip_mnt': '/mnt',
    };
    for (const [id, prefix] of Object.entries(chipMap)) {
        if (path.startsWith(prefix)) {
            document.getElementById(id)?.classList.add('active'); break;
        }
    }

    try {
        const resp = await fetch(`/browse?path=${encodeURIComponent(path)}`);
        const data = await resp.json();
        if (data.error) { log(`Browse error: ${data.error}`); return; }

        if (data.disk_free !== undefined) {
            const freeGB = (data.disk_free / 1e9).toFixed(1);
            const totalGB = (data.disk_total / 1e9).toFixed(1);
            document.getElementById('folderDiskInfo').innerText =
                `${freeGB} GB free of ${totalGB} GB on this volume`;
        }

        const list = document.getElementById('folderList');
        list.innerHTML = '';
        data.entries.forEach(entry => {
            const row = document.createElement('div');
            row.className = 'folder-entry' + (entry.type === 'file' ? ' is-file' : '');
            row.innerHTML = `<span class="icon">${entry.type === 'dir' ? '📁' : '📄'}</span><span>${entry.name}</span>`;
            if (entry.type === 'dir') row.addEventListener('click', () => browseTo(entry.path));
            list.appendChild(row);
        });
    } catch (e) {
        log(`Browse error: ${e}`);
    }
}

function confirmFolderSelection() {
    const path = document.getElementById('folderSelectedPath')?.innerText;
    if (!path) return;
    const sp = document.getElementById('save_path');
    if (sp) sp.value = path;
    sendCmd('set_save_path', { value: path });
    document.getElementById('folderModal').style.display = 'none';
    updateDiskSpace();
    log(`Save folder: ${path}`);
}

function createFolderPrompt() {
    const row = document.getElementById('newFolderRow');
    if (row) { row.style.display = 'flex'; document.getElementById('newFolderName')?.focus(); }
}

function cancelCreateFolder() {
    const row = document.getElementById('newFolderRow');
    if (row) row.style.display = 'none';
    const inp = document.getElementById('newFolderName');
    if (inp) inp.value = '';
}

function confirmCreateFolder() {
    const name = document.getElementById('newFolderName')?.value.trim();
    if (!name) { log("New folder: name is empty."); return; }
    sendCmd('create_folder', { path: folderBrowserCurrentPath, name });
    cancelCreateFolder();
}

function handleFolderCreated(data) {
    if (data.path) browseTo(data.path);
}

async function scanDrives() {
    const bar = document.getElementById('drivesBar');
    if (!bar) return;
    bar.querySelectorAll('.drive-chip.dynamic').forEach(c => c.remove());

    for (const base of ['/media/tim', '/media', '/mnt']) {
        try {
            const resp = await fetch(`/browse?path=${encodeURIComponent(base)}`);
            const data = await resp.json();
            if (data.error) continue;
            data.entries.filter(e => e.type === 'dir' && e.name !== '..').forEach(e => {
                const chip = document.createElement('button');
                chip.className = 'drive-chip dynamic';
                chip.innerText = `💾 ${e.name}`;
                chip.onclick = () => browseTo(e.path);
                bar.insertBefore(chip, bar.lastElementChild);
            });
        } catch (_) { }
    }
    log("Drives: rescanned external mounts.");
}
