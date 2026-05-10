// Native settings window + system tray, all in one HWND, all on the main thread.
//
// We use native-windows-gui (NWG) which is a thin Rust wrapper over Win32 HWND
// controls. Tray, menu, settings widgets, and the future 3D viz canvas all share
// the same Windows message loop — no thread juggling, no event-pump hacks.

use crate::{autostart, config, stats};
use nwd::NwgUi;
use nwg::NativeUi;
use std::cell::{Cell, RefCell};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

const W: i32 = 540;
const H: i32 = 776;

const AXIS_LABELS: [&str; 3] = ["raw X", "raw Y", "raw Z"];

#[derive(Default, NwgUi)]
pub struct App {
    // ---------- Window ----------
    #[nwg_control(
        size: (W, H),
        position: (300, 200),
        title: "SC2DSU — Steam Controller gyro to Cemuhook",
        flags: "WINDOW|VISIBLE|MINIMIZE_BOX"
    )]
    #[nwg_events(OnWindowClose: [App::on_close(SELF, EVT_DATA)], OnInit: [App::on_init])]
    window: nwg::Window,

    #[nwg_control(parent: window, interval: std::time::Duration::from_millis(100), active: true)]
    #[nwg_events(OnTimerTick: [App::refresh_stats])]
    timer: nwg::AnimationTimer,

    // Dedicated viz repaint at ~60 Hz. Separate from the stats timer so the text
    // labels (which reflow on every set_text) don't have to re-layout 60×/sec.
    #[nwg_control(parent: window, interval: std::time::Duration::from_millis(16), active: true)]
    #[nwg_events(OnTimerTick: [App::invalidate_viz])]
    viz_timer: nwg::AnimationTimer,

    // ---------- Tray ----------
    #[nwg_resource(source_bin: Some(BLUE_ICO_BYTES))]
    tray_icon: nwg::Icon,

    #[nwg_control(parent: window, icon: Some(&data.tray_icon), tip: Some("SC2DSU"))]
    #[nwg_events(MousePressLeftUp: [App::on_tray_left], OnContextMenu: [App::on_tray_right])]
    tray: nwg::TrayNotification,

    #[nwg_control(parent: window, popup: true)]
    tray_menu: nwg::Menu,

    #[nwg_control(parent: tray_menu, text: "Show settings")]
    #[nwg_events(OnMenuItemSelected: [App::show_window])]
    tray_show_item: nwg::MenuItem,

    #[nwg_control(parent: tray_menu)]
    tray_sep: nwg::MenuSeparator,

    #[nwg_control(parent: tray_menu, text: "Quit")]
    #[nwg_events(OnMenuItemSelected: [App::on_quit])]
    tray_quit_item: nwg::MenuItem,

    // ---------- Status frame ----------
    #[nwg_control(parent: window, position: (10, 6), size: (W - 20, 164))]
    status_frame: nwg::Frame,
    #[nwg_control(parent: status_frame, position: (10, 10), size: (260, 18), text: "Status")]
    lbl_status_hdr: nwg::Label,

    #[nwg_control(parent: status_frame, position: (12, 34), size: (500, 18), text: "Listening on:    binding…")]
    lbl_addr: nwg::Label,

    #[nwg_control(parent: status_frame, position: (12, 54), size: (500, 18), text: "Server id:       —")]
    lbl_id: nwg::Label,

    #[nwg_control(parent: status_frame, position: (12, 74), size: (500, 18), text: "Subscribers:     0     (controller idle)")]
    lbl_subs: nwg::Label,

    #[nwg_control(parent: status_frame, position: (12, 94), size: (500, 18), text: "IMU rate:        — Hz   →  packets sent —/s")]
    lbl_rate: nwg::Label,

    #[nwg_control(parent: status_frame, position: (12, 116), size: (500, 18), text: "gyro  (deg/s)  [    0    0    0]")]
    lbl_gyro: nwg::Label,

    #[nwg_control(parent: status_frame, position: (12, 136), size: (500, 18), text: "accel (g)      [0.000 0.000 0.000]")]
    lbl_accel: nwg::Label,

    // ---------- Gyro mapping frame ----------
    #[nwg_control(parent: window, position: (10, 178), size: (W - 20, 136))]
    gyro_frame: nwg::Frame,
    #[nwg_control(parent: gyro_frame, position: (10, 10), size: (260, 18), text: "Gyro axis mapping")]
    lbl_gyro_hdr: nwg::Label,

    #[nwg_control(parent: gyro_frame, position: (12, 34), size: (220, 18), text: "DSU X (Eden pitch)")]
    lbl_gx: nwg::Label,
    #[nwg_control(parent: gyro_frame, position: (240, 30), size: (90, 22))]
    #[nwg_events(OnComboxBoxSelection: [App::on_change])]
    cb_gx: nwg::ComboBox<&'static str>,
    #[nwg_control(parent: gyro_frame, position: (340, 32), size: (80, 18), text: "invert")]
    #[nwg_events(OnButtonClick: [App::on_change])]
    chk_gx: nwg::CheckBox,

    #[nwg_control(parent: gyro_frame, position: (12, 62), size: (220, 18), text: "DSU Y (Eden yaw)")]
    lbl_gy: nwg::Label,
    #[nwg_control(parent: gyro_frame, position: (240, 58), size: (90, 22))]
    #[nwg_events(OnComboxBoxSelection: [App::on_change])]
    cb_gy: nwg::ComboBox<&'static str>,
    #[nwg_control(parent: gyro_frame, position: (340, 60), size: (80, 18), text: "invert")]
    #[nwg_events(OnButtonClick: [App::on_change])]
    chk_gy: nwg::CheckBox,

    #[nwg_control(parent: gyro_frame, position: (12, 90), size: (220, 18), text: "DSU Z (Eden roll)")]
    lbl_gz: nwg::Label,
    #[nwg_control(parent: gyro_frame, position: (240, 86), size: (90, 22))]
    #[nwg_events(OnComboxBoxSelection: [App::on_change])]
    cb_gz: nwg::ComboBox<&'static str>,
    #[nwg_control(parent: gyro_frame, position: (340, 88), size: (80, 18), text: "invert")]
    #[nwg_events(OnButtonClick: [App::on_change])]
    chk_gz: nwg::CheckBox,

    // ---------- Accel mapping frame ----------
    #[nwg_control(parent: window, position: (10, 322), size: (W - 20, 156))]
    accel_frame: nwg::Frame,
    #[nwg_control(parent: accel_frame, position: (10, 10), size: (260, 18), text: "Accel axis mapping")]
    lbl_accel_hdr: nwg::Label,

    #[nwg_control(parent: accel_frame, position: (12, 34), size: (180, 22), text: "Copy from gyro")]
    #[nwg_events(OnButtonClick: [App::copy_gyro_to_accel])]
    btn_copy: nwg::Button,

    #[nwg_control(parent: accel_frame, position: (12, 62), size: (220, 18), text: "DSU X")]
    lbl_ax: nwg::Label,
    #[nwg_control(parent: accel_frame, position: (240, 58), size: (90, 22))]
    #[nwg_events(OnComboxBoxSelection: [App::on_change])]
    cb_ax: nwg::ComboBox<&'static str>,
    #[nwg_control(parent: accel_frame, position: (340, 60), size: (80, 18), text: "invert")]
    #[nwg_events(OnButtonClick: [App::on_change])]
    chk_ax: nwg::CheckBox,

    #[nwg_control(parent: accel_frame, position: (12, 90), size: (220, 18), text: "DSU Y")]
    lbl_ay: nwg::Label,
    #[nwg_control(parent: accel_frame, position: (240, 86), size: (90, 22))]
    #[nwg_events(OnComboxBoxSelection: [App::on_change])]
    cb_ay: nwg::ComboBox<&'static str>,
    #[nwg_control(parent: accel_frame, position: (340, 88), size: (80, 18), text: "invert")]
    #[nwg_events(OnButtonClick: [App::on_change])]
    chk_ay: nwg::CheckBox,

    #[nwg_control(parent: accel_frame, position: (12, 118), size: (220, 18), text: "DSU Z")]
    lbl_az: nwg::Label,
    #[nwg_control(parent: accel_frame, position: (240, 114), size: (90, 22))]
    #[nwg_events(OnComboxBoxSelection: [App::on_change])]
    cb_az: nwg::ComboBox<&'static str>,
    #[nwg_control(parent: accel_frame, position: (340, 116), size: (80, 18), text: "invert")]
    #[nwg_events(OnButtonClick: [App::on_change])]
    chk_az: nwg::CheckBox,

    // ---------- System frame ----------
    #[nwg_control(parent: window, position: (10, 486), size: (W - 20, 88))]
    sys_frame: nwg::Frame,
    #[nwg_control(parent: sys_frame, position: (10, 10), size: (260, 18), text: "System")]
    lbl_sys_hdr: nwg::Label,

    #[nwg_control(parent: sys_frame, position: (12, 34), size: (160, 18), text: "UDP port (next launch):")]
    lbl_port: nwg::Label,
    #[nwg_control(parent: sys_frame, position: (180, 32), size: (80, 22))]
    #[nwg_events(OnTextInput: [App::on_change])]
    edit_port: nwg::TextInput,

    #[nwg_control(parent: sys_frame, position: (12, 62), size: (250, 18), text: "Start with Windows (per-user)")]
    #[nwg_events(OnButtonClick: [App::on_autostart_toggle])]
    chk_autostart: nwg::CheckBox,

    #[nwg_control(parent: sys_frame, position: (270, 62), size: (240, 18), text: "Start minimized to tray")]
    #[nwg_events(OnButtonClick: [App::on_start_min_toggle])]
    chk_start_min: nwg::CheckBox,

    // ---------- 3D viz canvas (live wireframe driven by accel) ----------
    // ExternCanvas takes its parent as Option<&handle>, unlike most controls.
    #[nwg_control(parent: Some(&data.window), position: (10, 582), size: (W - 20, 152))]
    #[nwg_events(OnPaint: [App::on_viz_paint(SELF, EVT_DATA)])]
    viz_canvas: nwg::ExternCanvas,

    // ---------- Bottom bar ----------
    #[nwg_control(parent: window, position: (10, H - 36), size: (110, 26), text: "Hide to tray")]
    #[nwg_events(OnButtonClick: [App::hide_window])]
    btn_hide: nwg::Button,

    #[nwg_control(parent: window, position: (130, H - 36), size: (110, 26), text: "Recenter viz")]
    #[nwg_events(OnButtonClick: [App::on_recenter])]
    btn_recenter: nwg::Button,

    #[nwg_control(parent: window, position: (250, H - 36), size: (80, 26), text: "Quit")]
    #[nwg_events(OnButtonClick: [App::on_quit])]
    btn_quit: nwg::Button,

    #[nwg_control(parent: window, position: (340, H - 32), size: (200, 18), text: "")]
    lbl_save: nwg::Label,

    // ---------- Mutable state ----------
    shutdown: RefCell<Arc<AtomicBool>>,
    suppress_change: Cell<bool>,
    /// Set by the launcher when --tray was passed; in on_init we then hide the
    /// window for this launch regardless of the saved start_minimized config.
    start_min_requested: Cell<bool>,
    /// Atomic shared with the device thread: while this is true the controller
    /// is opened (so the viz has live samples) regardless of whether any DSU
    /// client is subscribed. We raise it whenever the settings window is
    /// visible and lower it when minimized to tray.
    ui_wants_device: RefCell<Arc<AtomicBool>>,
}

impl App {
    fn on_init(&self) {
        // Populate axis combos
        let items: Vec<&'static str> = AXIS_LABELS.to_vec();
        for cb in [
            &self.cb_gx,
            &self.cb_gy,
            &self.cb_gz,
            &self.cb_ax,
            &self.cb_ay,
            &self.cb_az,
        ] {
            cb.set_collection(items.clone());
        }
        self.suppress_change.set(true);
        self.populate_from_config();
        self.suppress_change.set(false);

        // Autostart checkbox state
        let on = autostart::is_enabled();
        self.chk_autostart.set_check_state(if on {
            nwg::CheckBoxState::Checked
        } else {
            nwg::CheckBoxState::Unchecked
        });

        // Start-minimized checkbox state from saved config.
        let cfg = config::snapshot();
        self.chk_start_min.set_check_state(if cfg.start_minimized {
            nwg::CheckBoxState::Checked
        } else {
            nwg::CheckBoxState::Unchecked
        });

        // Honor "start minimized" — either from saved config OR from the --tray
        // CLI flag (which the launcher pre-sets via the start_min cell). Otherwise
        // the window is visible from the get-go, so raise the device-want flag
        // for the viz.
        let hidden = cfg.start_minimized || self.start_min_requested.get();
        if hidden {
            self.window.set_visible(false);
        }
        if let Ok(b) = self.ui_wants_device.try_borrow() {
            b.store(!hidden, Ordering::Relaxed);
        }
    }

    fn populate_from_config(&self) {
        let cfg = config::snapshot();
        self.set_axis_widgets(&cfg.gyro.x, &self.cb_gx, &self.chk_gx);
        self.set_axis_widgets(&cfg.gyro.y, &self.cb_gy, &self.chk_gy);
        self.set_axis_widgets(&cfg.gyro.z, &self.cb_gz, &self.chk_gz);
        self.set_axis_widgets(&cfg.accel.x, &self.cb_ax, &self.chk_ax);
        self.set_axis_widgets(&cfg.accel.y, &self.cb_ay, &self.chk_ay);
        self.set_axis_widgets(&cfg.accel.z, &self.cb_az, &self.chk_az);
        self.edit_port.set_text(&cfg.port.to_string());
    }

    fn set_axis_widgets(
        &self,
        axis: &config::Axis,
        cb: &nwg::ComboBox<&'static str>,
        chk: &nwg::CheckBox,
    ) {
        cb.set_selection(Some((axis.source as usize).min(2)));
        chk.set_check_state(if axis.invert {
            nwg::CheckBoxState::Checked
        } else {
            nwg::CheckBoxState::Unchecked
        });
    }

    fn read_axis_widgets(
        &self,
        cb: &nwg::ComboBox<&'static str>,
        chk: &nwg::CheckBox,
    ) -> config::Axis {
        let source = cb.selection().unwrap_or(0).min(2) as u8;
        let invert = matches!(chk.check_state(), nwg::CheckBoxState::Checked);
        config::Axis::new(source, invert)
    }

    fn on_change(&self) {
        if self.suppress_change.get() {
            return;
        }
        let mut cfg = config::snapshot();
        cfg.gyro.x = self.read_axis_widgets(&self.cb_gx, &self.chk_gx);
        cfg.gyro.y = self.read_axis_widgets(&self.cb_gy, &self.chk_gy);
        cfg.gyro.z = self.read_axis_widgets(&self.cb_gz, &self.chk_gz);
        cfg.accel.x = self.read_axis_widgets(&self.cb_ax, &self.chk_ax);
        cfg.accel.y = self.read_axis_widgets(&self.cb_ay, &self.chk_ay);
        cfg.accel.z = self.read_axis_widgets(&self.cb_az, &self.chk_az);
        if let Ok(p) = self.edit_port.text().parse::<u16>() {
            cfg.port = p;
        }
        match config::update_and_save(cfg) {
            Ok(()) => self.lbl_save.set_text("saved."),
            Err(e) => self.lbl_save.set_text(&format!("save failed: {e}")),
        }
    }

    fn copy_gyro_to_accel(&self) {
        let cfg = config::snapshot();
        self.suppress_change.set(true);
        self.set_axis_widgets(&cfg.gyro.x, &self.cb_ax, &self.chk_ax);
        self.set_axis_widgets(&cfg.gyro.y, &self.cb_ay, &self.chk_ay);
        self.set_axis_widgets(&cfg.gyro.z, &self.cb_az, &self.chk_az);
        self.suppress_change.set(false);
        self.on_change();
    }

    fn on_start_min_toggle(&self) {
        let want = matches!(
            self.chk_start_min.check_state(),
            nwg::CheckBoxState::Checked
        );
        let mut cfg = config::snapshot();
        cfg.start_minimized = want;
        match config::update_and_save(cfg) {
            Ok(()) => self.lbl_save.set_text(if want {
                "will start minimized next launch."
            } else {
                "will start with window visible."
            }),
            Err(e) => self.lbl_save.set_text(&format!("save failed: {e}")),
        }
    }

    fn on_autostart_toggle(&self) {
        let want = matches!(
            self.chk_autostart.check_state(),
            nwg::CheckBoxState::Checked
        );
        let res = if want {
            autostart::enable()
        } else {
            autostart::disable()
        };
        match res {
            Ok(()) => self.lbl_save.set_text(if want {
                "autostart enabled."
            } else {
                "autostart disabled."
            }),
            Err(e) => self
                .lbl_save
                .set_text(&format!("autostart change failed: {e}")),
        }
    }

    fn refresh_stats(&self) {
        let s = stats::snapshot();
        self.lbl_addr.set_text(&format!(
            "Listening on:    {}",
            if s.bound_port == 0 {
                "binding…".to_string()
            } else {
                format!("0.0.0.0:{}", s.bound_port)
            }
        ));
        self.lbl_id
            .set_text(&format!("Server id:       0x{:08X}", s.server_id));
        self.lbl_subs.set_text(&format!(
            "Subscribers:     {}     ({})",
            s.subscribers,
            if s.device_active {
                "controller awake"
            } else {
                "controller idle"
            }
        ));
        self.lbl_rate.set_text(&format!(
            "IMU rate:        {:>6.1} Hz   →  packets sent {:>6.1}/s   reqs {:>4.1}/s",
            s.samples_per_sec, s.packets_per_sec, s.requests_per_sec,
        ));
        self.lbl_gyro.set_text(&format!(
            "gyro  (deg/s)  [{:>+8.1} {:>+8.1} {:>+8.1}]",
            s.last_gyro_dps[0], s.last_gyro_dps[1], s.last_gyro_dps[2]
        ));
        self.lbl_accel.set_text(&format!(
            "accel (g)      [{:>+6.3} {:>+6.3} {:>+6.3}]",
            s.last_accel_g[0], s.last_accel_g[1], s.last_accel_g[2]
        ));
    }

    fn invalidate_viz(&self) {
        use windows_sys::Win32::Graphics::Gdi::InvalidateRect;
        if let nwg::ControlHandle::Hwnd(hwnd) = self.viz_canvas.handle {
            unsafe {
                InvalidateRect(hwnd as _, std::ptr::null(), 0);
            }
        }
    }

    fn on_viz_paint(&self, evt: &nwg::EventData) {
        use windows_sys::Win32::Foundation::RECT;
        use windows_sys::Win32::Graphics::Gdi::{
            BitBlt, CreateCompatibleBitmap, CreateCompatibleDC, CreatePen, CreateSolidBrush,
            DeleteDC, DeleteObject, FillRect, LineTo, MoveToEx, PS_SOLID, SRCCOPY, SelectObject,
        };
        use windows_sys::Win32::UI::WindowsAndMessaging::GetClientRect;

        let pd = match evt {
            nwg::EventData::OnPaint(p) => p,
            _ => return,
        };
        let paint = pd.begin_paint();
        let screen_hdc: windows_sys::Win32::Graphics::Gdi::HDC = paint.hdc as _;
        let raw_hwnd = match self.viz_canvas.handle {
            nwg::ControlHandle::Hwnd(h) => h,
            _ => {
                pd.end_paint(&paint);
                return;
            }
        };
        let hwnd: windows_sys::Win32::Foundation::HWND = raw_hwnd as _;

        let mut rect: RECT = unsafe { std::mem::zeroed() };
        unsafe { GetClientRect(hwnd, &mut rect) };
        let w_px = rect.right - rect.left;
        let h_px = rect.bottom - rect.top;
        let w = w_px as f32;
        let h = h_px as f32;

        // Double-buffer: render to a memory DC, then BitBlt to the screen DC. This
        // eliminates the "white flash → fill → lines" sequence visible on direct
        // GDI draws, which reads as bad jitter even when the data is fresh.
        let mem_dc = unsafe { CreateCompatibleDC(screen_hdc) };
        let mem_bm = unsafe { CreateCompatibleBitmap(screen_hdc, w_px, h_px) };
        let old_bm = unsafe { SelectObject(mem_dc, mem_bm as _) };

        // Background fill — dark grey.
        unsafe {
            let bg = CreateSolidBrush(0x202020);
            FillRect(mem_dc, &rect, bg);
            let _ = DeleteObject(bg as _);
        }

        // Use the gyro-integrated orientation quaternion published by the DSU
        // thread. This shows real 3D rotation (including yaw) rather than only
        // the gravity-derived tilt. Drifts slowly over time — that's expected
        // until we add accel-based drift correction.
        let q = stats::snapshot().orientation;

        let cx = w * 0.5;
        let cy = h * 0.5;
        let scale = h * 0.32;
        let project = |v: [f32; 3]| -> (i32, i32) {
            let r = quat_rotate(q, v);
            ((cx + r[0] * scale) as i32, (cy - r[1] * scale) as i32)
        };

        let verts: [[f32; 3]; 8] = [
            [-1.0, -0.4, -1.0],
            [1.0, -0.4, -1.0],
            [1.0, 0.4, -1.0],
            [-1.0, 0.4, -1.0],
            [-1.0, -0.4, 1.0],
            [1.0, -0.4, 1.0],
            [1.0, 0.4, 1.0],
            [-1.0, 0.4, 1.0],
        ];
        let edges: [(usize, usize); 12] = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ];

        unsafe {
            let pen = CreatePen(PS_SOLID, 2, 0x60E080);
            let old = SelectObject(mem_dc, pen as _);
            for (a, b) in edges {
                let p0 = project(verts[a]);
                let p1 = project(verts[b]);
                MoveToEx(mem_dc, p0.0, p0.1, std::ptr::null_mut());
                LineTo(mem_dc, p1.0, p1.1);
            }
            SelectObject(mem_dc, old);
            let _ = DeleteObject(pen as _);
        }

        let axes: [([f32; 3], u32); 3] = [
            ([1.6, 0.0, 0.0], 0x0000FF),
            ([0.0, 1.6, 0.0], 0x00FF00),
            ([0.0, 0.0, 1.6], 0xFF0000),
        ];
        let origin = project([0.0, 0.0, 0.0]);
        for (v, color) in axes {
            unsafe {
                let pen = CreatePen(PS_SOLID, 3, color);
                let old = SelectObject(mem_dc, pen as _);
                MoveToEx(mem_dc, origin.0, origin.1, std::ptr::null_mut());
                let tip = project(v);
                LineTo(mem_dc, tip.0, tip.1);
                SelectObject(mem_dc, old);
                let _ = DeleteObject(pen as _);
            }
        }

        // Blit composed buffer to the screen, then clean up.
        unsafe {
            BitBlt(screen_hdc, 0, 0, w_px, h_px, mem_dc, 0, 0, SRCCOPY);
            SelectObject(mem_dc, old_bm);
            let _ = DeleteObject(mem_bm as _);
            let _ = DeleteDC(mem_dc);
        }

        pd.end_paint(&paint);
    }

    fn on_close(&self, evt: &nwg::EventData) {
        // Intercept the X button: hide to tray instead of exiting.
        if let nwg::EventData::OnWindowClose(close) = evt {
            close.close(false);
        }
        self.window.set_visible(false);
    }

    fn on_tray_left(&self) {
        // Toggle visibility on left-click of the tray icon.
        let visible = self.window.visible();
        self.window.set_visible(!visible);
        if !visible {
            self.window.restore();
        }
    }

    fn on_tray_right(&self) {
        // Show the menu where the cursor is.
        let (x, y) = nwg::GlobalCursor::position();
        self.tray_menu.popup(x, y);
    }

    fn show_window(&self) {
        self.window.set_visible(true);
        self.window.restore();
        if let Ok(b) = self.ui_wants_device.try_borrow() {
            b.store(true, Ordering::Relaxed);
        }
    }

    fn hide_window(&self) {
        self.window.set_visible(false);
        if let Ok(b) = self.ui_wants_device.try_borrow() {
            b.store(false, Ordering::Relaxed);
        }
    }

    fn on_recenter(&self) {
        // Signal the DSU thread to reset its orientation integrator to identity
        // on the next IMU sample. The published quaternion then propagates to
        // the viz on the next 60 Hz redraw.
        stats::RECENTER_REQUEST.store(true, Ordering::Relaxed);
        self.lbl_save.set_text("recentered.");
    }

    fn on_quit(&self) {
        if let Ok(s) = self.shutdown.try_borrow() {
            s.store(true, Ordering::Relaxed);
        }
        nwg::stop_thread_dispatch();
    }
}

pub fn run(
    shutdown: Arc<AtomicBool>,
    ui_wants_device: Arc<AtomicBool>,
    start_minimized: bool,
) -> Result<(), String> {
    nwg::init().map_err(|e| format!("nwg::init: {e}"))?;
    nwg::Font::set_global_family("Segoe UI").ok();

    let app = App {
        shutdown: RefCell::new(shutdown),
        ui_wants_device: RefCell::new(ui_wants_device),
        start_min_requested: Cell::new(start_minimized),
        ..Default::default()
    };
    let _ui = App::build_ui(app).map_err(|e| format!("build_ui: {e}"))?;
    nwg::dispatch_thread_events();
    Ok(())
}

// Tiny embedded 16x16 ICO — just a solid blue square. Generated once and embedded
// so we don't ship any external icon files.
const BLUE_ICO_BYTES: &[u8] = include_bytes!("../assets/tray.ico");

// ---------- Math helpers ----------

/// Rotate vector `v` by quaternion `q = (w, x, y, z)` using the standard
/// 2(q_xyz × (q_xyz × v + w·v)) formulation — fewer multiplications than going
/// through a 3×3 matrix and avoids any allocation.
fn quat_rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let qx = [x, y, z];
    let c1 = cross(qx, v);
    let t = [c1[0] + w * v[0], c1[1] + w * v[1], c1[2] + w * v[2]];
    let c2 = cross(qx, t);
    [v[0] + 2.0 * c2[0], v[1] + 2.0 * c2[1], v[2] + 2.0 * c2[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
