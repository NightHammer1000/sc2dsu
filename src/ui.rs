use crate::{autostart, config, stats};
use nwd::NwgUi;
use nwg::NativeUi;
use std::cell::Cell;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use windows_sys::Win32::Graphics::Gdi::HDC;

const W: i32 = 540;
const H: i32 = 810;

const AXIS_LABELS: [&str; 3] = ["raw X", "raw Y", "raw Z"];

const VIZ_BG_COLOR: u32 = 0x0020_2020;
const VIZ_EDGE_COLOR: u32 = 0x0060_E080;
const AXIS_X_COLOR: u32 = 0x0000_00FF;
const AXIS_Y_COLOR: u32 = 0x0000_FF00;
const AXIS_Z_COLOR: u32 = 0x00FF_0000;

fn checkbox_state(on: bool) -> nwg::CheckBoxState {
    if on {
        nwg::CheckBoxState::Checked
    } else {
        nwg::CheckBoxState::Unchecked
    }
}

fn is_checked(chk: &nwg::CheckBox) -> bool {
    matches!(chk.check_state(), nwg::CheckBoxState::Checked)
}

#[derive(Default, NwgUi)]
pub struct App {
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

    #[nwg_control(parent: window, interval: std::time::Duration::from_millis(16), active: true)]
    #[nwg_events(OnTimerTick: [App::invalidate_viz])]
    viz_timer: nwg::AnimationTimer,

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

    #[nwg_control(parent: window, position: (10, 178), size: (W - 20, 136))]
    gyro_frame: nwg::Frame,
    #[nwg_control(parent: gyro_frame, position: (10, 10), size: (260, 18), text: "Gyro axis mapping")]
    lbl_gyro_hdr: nwg::Label,

    #[nwg_control(parent: gyro_frame, position: (290, 8), size: (220, 22), text: "Auto-calibrate bias")]
    #[nwg_events(OnButtonClick: [App::on_auto_cal_toggle])]
    chk_auto_cal: nwg::CheckBox,

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

    #[nwg_control(parent: window, position: (10, 486), size: (W - 20, 116))]
    sys_frame: nwg::Frame,
    #[nwg_control(parent: sys_frame, position: (10, 10), size: (260, 18), text: "System")]
    lbl_sys_hdr: nwg::Label,

    #[nwg_control(parent: sys_frame, position: (12, 34), size: (160, 18), text: "UDP port (next launch):")]
    lbl_port: nwg::Label,
    #[nwg_control(parent: sys_frame, position: (180, 32), size: (80, 22))]
    #[nwg_events(OnTextInput: [App::on_change])]
    edit_port: nwg::TextInput,

    #[nwg_control(parent: sys_frame, position: (280, 34), size: (200, 18), text: "Open to network")]
    #[nwg_events(OnButtonClick: [App::on_expose_toggle])]
    chk_expose: nwg::CheckBox,

    #[nwg_control(parent: sys_frame, position: (12, 62), size: (250, 18), text: "Start with Windows (per-user)")]
    #[nwg_events(OnButtonClick: [App::on_autostart_toggle])]
    chk_autostart: nwg::CheckBox,

    #[nwg_control(parent: sys_frame, position: (270, 62), size: (240, 18), text: "Start minimized to tray")]
    #[nwg_events(OnButtonClick: [App::on_start_min_toggle])]
    chk_start_min: nwg::CheckBox,

    #[nwg_control(parent: sys_frame, position: (12, 90), size: (320, 18), text: "Hide to tray on window close (don't quit)")]
    #[nwg_events(OnButtonClick: [App::on_close_to_tray_toggle])]
    chk_close_to_tray: nwg::CheckBox,

    #[nwg_control(parent: sys_frame, position: (360, 86), size: (150, 24), text: "Restore defaults")]
    #[nwg_events(OnButtonClick: [App::on_restore_defaults])]
    btn_restore: nwg::Button,

    #[nwg_control(parent: Some(&data.window), position: (10, 610), size: (W - 20, 152))]
    #[nwg_events(OnPaint: [App::on_viz_paint(SELF, EVT_DATA)])]
    viz_canvas: nwg::ExternCanvas,

    #[nwg_control(parent: window, position: (10, H - 36), size: (110, 26), text: "Hide to tray")]
    #[nwg_events(OnButtonClick: [App::hide_window])]
    btn_hide: nwg::Button,

    #[nwg_control(parent: window, position: (130, H - 36), size: (110, 26), text: "Recalibrate")]
    #[nwg_events(OnButtonClick: [App::on_recenter])]
    btn_recenter: nwg::Button,

    #[nwg_control(parent: window, position: (250, H - 36), size: (80, 26), text: "Quit")]
    #[nwg_events(OnButtonClick: [App::on_quit])]
    btn_quit: nwg::Button,

    #[nwg_control(parent: window, position: (340, H - 32), size: (200, 18), text: "")]
    lbl_save: nwg::Label,

    shutdown: Arc<AtomicBool>,
    suppress_change: Cell<bool>,
    start_min_requested: Cell<bool>,
    ui_wants_device: Arc<AtomicBool>,
}

impl App {
    fn on_init(&self) {
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

        self.chk_autostart
            .set_check_state(checkbox_state(autostart::is_enabled()));

        let cfg = config::snapshot();
        self.chk_start_min
            .set_check_state(checkbox_state(cfg.start_minimized));
        self.chk_expose
            .set_check_state(checkbox_state(cfg.expose_to_network));
        self.chk_close_to_tray
            .set_check_state(checkbox_state(cfg.close_to_tray));
        self.chk_auto_cal
            .set_check_state(checkbox_state(cfg.auto_calibrate));

        let hidden = cfg.start_minimized || self.start_min_requested.get();
        self.set_window_shown(!hidden);
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
        chk.set_check_state(checkbox_state(axis.invert));
    }

    fn read_axis_widgets(
        &self,
        cb: &nwg::ComboBox<&'static str>,
        chk: &nwg::CheckBox,
    ) -> config::Axis {
        let source = cb.selection().unwrap_or(0).min(2) as u8;
        config::Axis::new(source, is_checked(chk))
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
        let port_text = self.edit_port.text();
        let mut note: &str = "saved.";
        match port_text.parse::<u16>() {
            Ok(p) => cfg.port = p,
            Err(_) if port_text.trim().is_empty() => {}
            Err(_) => note = "saved — port field ignored (need a number 0-65535).",
        }
        match config::update_and_save(cfg) {
            Ok(()) => self.lbl_save.set_text(note),
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
        let want = is_checked(&self.chk_start_min);
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

    fn on_expose_toggle(&self) {
        let want = is_checked(&self.chk_expose);
        let mut cfg = config::snapshot();
        cfg.expose_to_network = want;
        match config::update_and_save(cfg) {
            Ok(()) => self.lbl_save.set_text(if want {
                "open to network (next launch)."
            } else {
                "127.0.0.1 only (next launch)."
            }),
            Err(e) => self.lbl_save.set_text(&format!("save failed: {e}")),
        }
    }

    fn on_auto_cal_toggle(&self) {
        let want = is_checked(&self.chk_auto_cal);
        let mut cfg = config::snapshot();
        cfg.auto_calibrate = want;
        match config::update_and_save(cfg) {
            Ok(()) => self.lbl_save.set_text(if want {
                "auto-calibration enabled."
            } else {
                "auto-calibration disabled — raw gyro passthrough."
            }),
            Err(e) => self.lbl_save.set_text(&format!("save failed: {e}")),
        }
    }

    fn on_close_to_tray_toggle(&self) {
        let want = is_checked(&self.chk_close_to_tray);
        let mut cfg = config::snapshot();
        cfg.close_to_tray = want;
        match config::update_and_save(cfg) {
            Ok(()) => self.lbl_save.set_text(if want {
                "close button now hides to tray."
            } else {
                "close button now quits."
            }),
            Err(e) => self.lbl_save.set_text(&format!("save failed: {e}")),
        }
    }

    fn on_autostart_toggle(&self) {
        let want = is_checked(&self.chk_autostart);
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
        let host = config::bind_host(config::snapshot().expose_to_network);
        self.lbl_addr.set_text(&format!(
            "Listening on:    {}",
            if s.server.bound_port == 0 {
                "binding…".to_string()
            } else {
                format!("{host}:{}", s.server.bound_port)
            }
        ));
        self.lbl_id
            .set_text(&format!("Server id:       0x{:08X}", s.server.server_id));
        let device_state = if s.server.device_active {
            "controller awake"
        } else {
            "controller idle"
        };
        let cal_state: String = if !s.calibration.active {
            "gyro: off".into()
        } else if s.calibration.steady {
            format!("gyro: locked {:.0}%", s.calibration.confidence * 100.0)
        } else {
            "gyro: calibrating…".into()
        };
        self.lbl_subs.set_text(&format!(
            "Subscribers:     {}     ({device_state} · {cal_state})",
            s.server.subscribers,
        ));
        self.lbl_rate.set_text(&format!(
            "IMU rate:        {:>6.1} Hz   →  packets sent {:>6.1}/s   reqs {:>4.1}/s",
            s.server.samples_per_sec, s.server.packets_per_sec, s.server.requests_per_sec,
        ));
        self.lbl_gyro.set_text(&format!(
            "gyro  (deg/s)  [{:>+8.1} {:>+8.1} {:>+8.1}]",
            s.motion.last_gyro_dps[0], s.motion.last_gyro_dps[1], s.motion.last_gyro_dps[2]
        ));
        self.lbl_accel.set_text(&format!(
            "accel (g)      [{:>+6.3} {:>+6.3} {:>+6.3}]",
            s.motion.last_accel_g[0], s.motion.last_accel_g[1], s.motion.last_accel_g[2]
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
            BitBlt, CreateCompatibleBitmap, CreateCompatibleDC, CreateSolidBrush, DeleteDC,
            DeleteObject, FillRect, SRCCOPY, SelectObject,
        };
        use windows_sys::Win32::UI::WindowsAndMessaging::GetClientRect;

        let pd = match evt {
            nwg::EventData::OnPaint(p) => p,
            _ => return,
        };
        let paint = pd.begin_paint();
        let screen_hdc: HDC = paint.hdc as _;
        let raw_hwnd = match self.viz_canvas.handle {
            nwg::ControlHandle::Hwnd(h) => h,
            _ => {
                pd.end_paint(&paint);
                return;
            }
        };
        let hwnd: windows_sys::Win32::Foundation::HWND = raw_hwnd as _;

        // SAFETY: RECT is a plain struct of i32 fields; an all-zero value is valid.
        let mut rect: RECT = unsafe { std::mem::zeroed() };
        // SAFETY: hwnd is a live window handle from the canvas; &mut rect is a valid out-pointer.
        unsafe { GetClientRect(hwnd, &mut rect) };
        let w_px = rect.right - rect.left;
        let h_px = rect.bottom - rect.top;
        let cx = (w_px as f32) * 0.5;
        let cy = (h_px as f32) * 0.5;
        let scale = (h_px as f32) * 0.32;

        let q = stats::snapshot().motion.orientation;
        let project = |v: [f32; 3]| -> (i32, i32) {
            let r = quat_rotate(q, v);
            ((cx + r[0] * scale) as i32, (cy - r[1] * scale) as i32)
        };

        // SAFETY: screen_hdc is the live paint DC. Every GDI object created in this block
        // (the offscreen DC, its bitmap, the background brush) is selected/used and then
        // restored or deleted before the block returns, so no handles leak.
        unsafe {
            let mem_dc = CreateCompatibleDC(screen_hdc);
            let mem_bm = CreateCompatibleBitmap(screen_hdc, w_px, h_px);
            let old_bm = SelectObject(mem_dc, mem_bm as _);

            let bg = CreateSolidBrush(VIZ_BG_COLOR);
            FillRect(mem_dc, &rect, bg);
            let _ = DeleteObject(bg as _);

            draw_wireframe_cube(mem_dc, &project);
            draw_axes_gizmo(mem_dc, &project);

            BitBlt(screen_hdc, 0, 0, w_px, h_px, mem_dc, 0, 0, SRCCOPY);
            SelectObject(mem_dc, old_bm);
            let _ = DeleteObject(mem_bm as _);
            let _ = DeleteDC(mem_dc);
        }

        pd.end_paint(&paint);
    }

    fn set_window_shown(&self, shown: bool) {
        self.window.set_visible(shown);
        if shown {
            self.window.restore();
        }
        self.ui_wants_device.store(shown, Ordering::Relaxed);
    }

    fn on_close(&self, evt: &nwg::EventData) {
        if let nwg::EventData::OnWindowClose(close) = evt {
            close.close(false);
        }
        if config::snapshot().close_to_tray {
            self.set_window_shown(false);
        } else {
            self.on_quit();
        }
    }

    fn on_tray_left(&self) {
        self.set_window_shown(!self.window.visible());
    }

    fn on_tray_right(&self) {
        let (x, y) = nwg::GlobalCursor::position();
        self.tray_menu.popup(x, y);
    }

    fn show_window(&self) {
        self.set_window_shown(true);
    }

    fn hide_window(&self) {
        self.set_window_shown(false);
    }

    fn on_restore_defaults(&self) {
        let prompt = nwg::MessageParams {
            title: "Restore defaults",
            content: "Reset all settings to defaults?",
            buttons: nwg::MessageButtons::YesNo,
            icons: nwg::MessageIcons::Warning,
        };
        if !matches!(
            nwg::modal_message(&self.window, &prompt),
            nwg::MessageChoice::Yes
        ) {
            return;
        }
        match config::update_and_save(config::Config::DEFAULT) {
            Ok(()) => {
                let cfg = config::snapshot();
                self.suppress_change.set(true);
                self.populate_from_config();
                self.chk_start_min
                    .set_check_state(checkbox_state(cfg.start_minimized));
                self.chk_expose
                    .set_check_state(checkbox_state(cfg.expose_to_network));
                self.chk_close_to_tray
                    .set_check_state(checkbox_state(cfg.close_to_tray));
                self.chk_auto_cal
                    .set_check_state(checkbox_state(cfg.auto_calibrate));
                self.suppress_change.set(false);
                self.lbl_save.set_text("restored defaults.");
            }
            Err(e) => self.lbl_save.set_text(&format!("restore failed: {e}")),
        }
    }

    fn on_recenter(&self) {
        stats::RECENTER_REQUEST.store(true, Ordering::Relaxed);
        stats::RECALIBRATE_REQUEST.store(true, Ordering::Relaxed);
        self.lbl_save.set_text("recalibrating gyro.");
    }

    fn on_quit(&self) {
        self.shutdown.store(true, Ordering::Relaxed);
        nwg::stop_thread_dispatch();
    }
}

pub fn run(
    shutdown: Arc<AtomicBool>,
    ui_wants_device: Arc<AtomicBool>,
    start_minimized: bool,
) -> Result<(), String> {
    nwg::init().map_err(|e| format!("nwg::init: {e}"))?;
    set_global_font()?;

    let app = App {
        shutdown,
        ui_wants_device,
        start_min_requested: Cell::new(start_minimized),
        ..Default::default()
    };
    let _ui = App::build_ui(app).map_err(|e| format!("build_ui: {e}"))?;
    nwg::dispatch_thread_events();
    Ok(())
}

fn set_global_font() -> Result<(), String> {
    let mut font = nwg::Font::default();
    nwg::Font::builder()
        .family("Segoe UI")
        .size(16)
        .build(&mut font)
        .map_err(|e| format!("font build: {e}"))?;
    nwg::Font::set_global_default(Some(font));
    Ok(())
}

const BLUE_ICO_BYTES: &[u8] = include_bytes!("../assets/tray.ico");

fn draw_wireframe_cube(dc: HDC, project: &dyn Fn([f32; 3]) -> (i32, i32)) {
    use windows_sys::Win32::Graphics::Gdi::{
        CreatePen, DeleteObject, LineTo, MoveToEx, PS_SOLID, SelectObject,
    };
    const VERTS: [[f32; 3]; 8] = [
        [-1.0, -0.4, -1.0],
        [1.0, -0.4, -1.0],
        [1.0, 0.4, -1.0],
        [-1.0, 0.4, -1.0],
        [-1.0, -0.4, 1.0],
        [1.0, -0.4, 1.0],
        [1.0, 0.4, 1.0],
        [-1.0, 0.4, 1.0],
    ];
    const EDGES: [(usize, usize); 12] = [
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
    // SAFETY: dc is a valid HDC owned by the caller; the pen is created, selected,
    // restored, and deleted within this block, leaving the DC as it was found.
    unsafe {
        let pen = CreatePen(PS_SOLID, 2, VIZ_EDGE_COLOR);
        let old = SelectObject(dc, pen as _);
        for (a, b) in EDGES {
            let p0 = project(VERTS[a]);
            let p1 = project(VERTS[b]);
            MoveToEx(dc, p0.0, p0.1, std::ptr::null_mut());
            LineTo(dc, p1.0, p1.1);
        }
        SelectObject(dc, old);
        let _ = DeleteObject(pen as _);
    }
}

fn draw_axes_gizmo(dc: HDC, project: &dyn Fn([f32; 3]) -> (i32, i32)) {
    use windows_sys::Win32::Graphics::Gdi::{
        CreatePen, DeleteObject, LineTo, MoveToEx, PS_SOLID, SelectObject,
    };
    let axes: [([f32; 3], u32); 3] = [
        ([1.6, 0.0, 0.0], AXIS_X_COLOR),
        ([0.0, 1.6, 0.0], AXIS_Y_COLOR),
        ([0.0, 0.0, 1.6], AXIS_Z_COLOR),
    ];
    let origin = project([0.0, 0.0, 0.0]);
    for (v, color) in axes {
        // SAFETY: dc is a valid HDC owned by the caller; each pen is created, selected,
        // restored, and deleted within this iteration.
        unsafe {
            let pen = CreatePen(PS_SOLID, 3, color);
            let old = SelectObject(dc, pen as _);
            MoveToEx(dc, origin.0, origin.1, std::ptr::null_mut());
            let tip = project(v);
            LineTo(dc, tip.0, tip.1);
            SelectObject(dc, old);
            let _ = DeleteObject(pen as _);
        }
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: [f32; 3], b: [f32; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-5)
    }

    #[test]
    fn quat_rotate_identity_is_noop() {
        let v = [0.3, -1.2, 4.0];
        assert!(approx_eq(quat_rotate([1.0, 0.0, 0.0, 0.0], v), v));
    }

    #[test]
    fn quat_rotate_90deg_about_z_maps_x_to_y() {
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let r = quat_rotate([s, 0.0, 0.0, s], [1.0, 0.0, 0.0]);
        assert!(approx_eq(r, [0.0, 1.0, 0.0]), "got {r:?}");
    }

    #[test]
    fn cross_of_basis_vectors() {
        assert_eq!(cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]);
        assert_eq!(cross([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]);
    }
}
