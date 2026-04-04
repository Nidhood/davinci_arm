#include "davinci_arm_gui/app/main_window.hpp"

#include "davinci_arm_gui/core/models/domain.hpp"
#include "davinci_arm_gui/core/models/theme_id.hpp"
#include "davinci_arm_gui/ui/pages/calibration_page.hpp"
#include "davinci_arm_gui/ui/pages/control_panel_page.hpp"
#include "davinci_arm_gui/ui/pages/dashboard_page.hpp"
#include "davinci_arm_gui/ui/pages/drawing_page.hpp"
#include "davinci_arm_gui/ui/style/theme_manager.hpp"

#include <QAction>
#include <QApplication>
#include <QCursor>
#include <QGuiApplication>
#include <QIcon>
#include <QLabel>
#include <QMenu>
#include <QScreen>
#include <QShowEvent>
#include <QStackedWidget>
#include <QStatusBar>
#include <QStyle>
#include <QTabBar>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>

#include <algorithm>

namespace davinci_arm::app {
using davinci_arm::models::Domain;

namespace {
constexpr int kStreamStaleMs = 1000;
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setWindowTitle("DavinciArm HMI");

    QIcon app_icon(":/icons/davinci_arm_icon.png");
    if (!app_icon.isNull()) {
        setWindowIcon(app_icon);
        QApplication::setWindowIcon(app_icon);
    }

    setMinimumSize(1200, 760);

    buildUi_();
    buildTopBar_();
    connectPageSignals_();
    setupStatusBar_();
    setupScreenTracking_();

    connect(&davinci_arm::ui::style::ThemeManager::instance(),
            &davinci_arm::ui::style::ThemeManager::themeChanged,
            this,
            &MainWindow::onThemeChanged_);

    if (top_tabs_) top_tabs_->setCurrentIndex(0);
    onNavChanged_(0);
}

void MainWindow::buildUi_() {
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    stack_ = new QStackedWidget(central);
    stack_->setObjectName("stack");

    dashboard_ = new davinci_arm::ui::pages::DashboardPage(stack_);
    control_ = new davinci_arm::ui::pages::ControlPanelPage(stack_);
    calibration_ = new davinci_arm::ui::pages::CalibrationPage(stack_);
    drawing_ = new davinci_arm::ui::pages::DrawingPage(stack_);

    stack_->addWidget(dashboard_);
    stack_->addWidget(control_);
    stack_->addWidget(calibration_);
    stack_->addWidget(drawing_);

    root->addWidget(stack_, 1);
}

void MainWindow::buildTopBar_() {
    topbar_ = new QToolBar(this);
    topbar_->setObjectName("topbar");
    topbar_->setMovable(false);
    topbar_->setFloatable(false);
    topbar_->setIconSize(QSize(18, 18));

    theme_btn_ = new QToolButton(this);
    theme_btn_->setObjectName("themeButton");
    theme_btn_->setAutoRaise(true);
    theme_btn_->setPopupMode(QToolButton::InstantPopup);
    theme_btn_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    theme_btn_->setIcon(style()->standardIcon(QStyle::SP_DesktopIcon));
    theme_btn_->setText("Theme");

    auto* theme_menu = new QMenu(theme_btn_);
    theme_menu->setObjectName("themeMenu");

    auto addTheme = [&](const QString& name, davinci_arm::models::ThemeId id) {
        QAction* action = theme_menu->addAction(name);
        action->setData(static_cast<int>(id));
        connect(action, &QAction::triggered, this, &MainWindow::onThemeAction_);
    };

    addTheme("Light", davinci_arm::models::ThemeId::Light);
    addTheme("Dark", davinci_arm::models::ThemeId::Dark);
    addTheme("MATLAB", davinci_arm::models::ThemeId::MATLAB);
    addTheme("Tron Evolution", davinci_arm::models::ThemeId::TronEvolution);
    addTheme("Tron Ares", davinci_arm::models::ThemeId::TronAres);
    addTheme("Cyberpunk Neon", davinci_arm::models::ThemeId::CyberpunkNeon);

    theme_btn_->setMenu(theme_menu);

    top_tabs_ = new QTabBar(this);
    top_tabs_->setObjectName("topTabs");
    top_tabs_->setExpanding(false);
    top_tabs_->setMovable(false);
    top_tabs_->setElideMode(Qt::ElideNone);
    top_tabs_->setUsesScrollButtons(false);
    top_tabs_->setDrawBase(false);
    top_tabs_->addTab("Dashboard");
    top_tabs_->addTab("Control Panel");
    top_tabs_->addTab("Calibration");
    top_tabs_->addTab("Drawing");

    connect(top_tabs_, &QTabBar::currentChanged, this, [this](int idx) {
        onNavChanged_(idx);
    });

    addToolBar(Qt::TopToolBarArea, topbar_);
    topbar_->addWidget(top_tabs_);

    auto* spacer = new QWidget(this);
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    topbar_->addWidget(spacer);
    topbar_->addWidget(theme_btn_);

    updateTopBarStyle_();
}

void MainWindow::onThemeAction_() {
    auto* action = qobject_cast<QAction*>(sender());
    if (!action) return;

    const auto id = static_cast<davinci_arm::models::ThemeId>(action->data().toInt());
    davinci_arm::ui::style::ThemeManager::instance().apply(id);
}

void MainWindow::connectPageSignals_() {
    connect(control_,
            &davinci_arm::ui::pages::ControlPanelPage::refAngleChanged,
            this,
            &MainWindow::refAngleChanged);

    connect(control_,
            &davinci_arm::ui::pages::ControlPanelPage::pwmChanged,
            this,
            &MainWindow::pwmChanged);

    connect(control_,
            &davinci_arm::ui::pages::ControlPanelPage::autoModeChanged,
            this,
    [this](bool auto_mode) {
        updateModeIndicator_(auto_mode);
        emit autoModeChanged(auto_mode);
    });

    connect(control_,
            &davinci_arm::ui::pages::ControlPanelPage::stopRequested,
            this,
            &MainWindow::stopRequested);
}

void MainWindow::setupStatusBar_() {
    status_connection_ = new QLabel("Disconnected");
    status_connection_->setObjectName("statusLabel");

    status_mode_ = new QLabel("Mode: Manual");
    status_mode_->setObjectName("statusLabel");

    status_real_ = new QLabel("REAL: --");
    status_real_->setObjectName("statusLabel");

    status_sim_ = new QLabel("SIM: --");
    status_sim_->setObjectName("statusLabel");

    auto createSeparator = [this]() {
        auto* separator = new QLabel(" | ");
        separator->setObjectName("statusSeparator");
        return separator;
    };

    statusBar()->setObjectName("statusBar");
    statusBar()->setSizeGripEnabled(false);
    statusBar()->addPermanentWidget(status_connection_);
    statusBar()->addPermanentWidget(createSeparator());
    statusBar()->addPermanentWidget(status_mode_);
    statusBar()->addPermanentWidget(createSeparator());
    statusBar()->addPermanentWidget(status_real_);
    statusBar()->addPermanentWidget(createSeparator());
    statusBar()->addPermanentWidget(status_sim_);

    updateStatusBarStyle_();

    status_timer_ = new QTimer(this);
    status_timer_->setInterval(200);
    status_timer_->setTimerType(Qt::CoarseTimer);
    connect(status_timer_, &QTimer::timeout, this, &MainWindow::updateStatusLabels_);
    status_timer_->start();
}

void MainWindow::updateStatusBarStyle_() {
    const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();

    const QString style_sheet = QString(
                                    "QStatusBar#statusBar { "
                                    "background-color: %1 !important; "
                                    "color: %2 !important; "
                                    "border-top: 1px solid %3 !important; "
                                    "padding: 7px 20px !important; "
                                    "min-height: 20px !important; "
                                    "} "
                                    "QLabel#statusLabel { "
                                    "color: %2 !important; "
                                    "font-weight: 600 !important; "
                                    "padding: 2px 8px !important; "
                                    "background-color: transparent !important; "
                                    "} "
                                    "QLabel#statusSeparator { "
                                    "color: %4 !important; "
                                    "font-weight: 400 !important; "
                                    "background-color: transparent !important; "
                                    "} ")
                                .arg(spec.panel.name())
                                .arg(spec.text.name())
                                .arg(spec.accent.name())
                                .arg(spec.text_muted.name());

    statusBar()->setStyleSheet(style_sheet);
}

void MainWindow::updateTopBarStyle_() {
    const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();

    const QString toolbar_style = QString(
                                      "QToolBar#topbar { "
                                      "background-color: %1 !important; "
                                      "border: none !important; "
                                      "border-bottom: 1px solid %2 !important; "
                                      "padding: 0px !important; "
                                      "spacing: 0px !important; "
                                      "min-height: 40px !important; "
                                      "} "
                                      "QToolBar#topbar::separator { "
                                      "background: transparent !important; "
                                      "width: 0px !important; "
                                      "} "
                                      "QToolButton#themeButton { "
                                      "background-color: transparent !important; "
                                      "color: %3 !important; "
                                      "border: none !important; "
                                      "padding: 8px 12px !important; "
                                      "border-radius: 4px !important; "
                                      "} "
                                      "QToolButton#themeButton:hover { "
                                      "background-color: %4 !important; "
                                      "color: %5 !important; "
                                      "} "
                                      "QToolButton#themeButton::menu-indicator { "
                                      "image: none !important; "
                                      "} ")
                                  .arg(spec.panel.name())
                                  .arg(spec.accent.name())
                                  .arg(spec.text.name())
                                  .arg(spec.accent.lighter(130).name())
                                  .arg(spec.bg.name());

    topbar_->setStyleSheet(toolbar_style);

    const QString tab_style = QString(
                                  "QTabBar#topTabs { background: transparent !important; border: none !important; } "
                                  "QTabBar#topTabs::tab { "
                                  "background: transparent !important; "
                                  "color: %1 !important; "
                                  "border: none !important; "
                                  "padding: 10px 20px !important; "
                                  "margin: 0px !important; "
                                  "font-weight: 600 !important; "
                                  "} "
                                  "QTabBar#topTabs::tab:hover { "
                                  "background: %2 !important; "
                                  "color: %3 !important; "
                                  "font-weight: 400 !important; "
                                  "border-top-left-radius: 8px !important; "
                                  "border-top-right-radius: 8px !important; "
                                  "} "
                                  "QTabBar#topTabs::tab:selected { "
                                  "background: %4 !important; "
                                  "color: %2 !important; "
                                  "font-weight: 600 !important; "
                                  "border: 1px solid %2 !important; "
                                  "border-bottom: 3px solid %4 !important; "
                                  "border-top-left-radius: 8px !important; "
                                  "border-top-right-radius: 8px !important; "
                                  "margin-bottom: -2px !important; "
                                  "padding-bottom: 9px !important; "
                                  "} ")
                              .arg(spec.text_muted.name())
                              .arg(spec.accent.name())
                              .arg(spec.bg.name())
                              .arg(spec.bg.name());

    top_tabs_->setStyleSheet(tab_style);

    const QString menu_style = QString(
                                   "QMenu#themeMenu { "
                                   "background-color: %1 !important; "
                                   "color: %2 !important; "
                                   "border: 1px solid %3 !important; "
                                   "padding: 5px !important; "
                                   "} "
                                   "QMenu#themeMenu::item { "
                                   "background-color: transparent !important; "
                                   "color: %2 !important; "
                                   "padding: 8px 20px !important; "
                                   "border-radius: 4px !important; "
                                   "} "
                                   "QMenu#themeMenu::item:hover { "
                                   "background-color: %3 !important; "
                                   "color: %4 !important; "
                                   "} "
                                   "QMenu#themeMenu::item:selected { "
                                   "background-color: %3 !important; "
                                   "color: %4 !important; "
                                   "} ")
                               .arg(spec.panel.name())
                               .arg(spec.text.name())
                               .arg(spec.accent.name())
                               .arg(spec.bg.name());

    if (theme_btn_ && theme_btn_->menu()) {
        theme_btn_->menu()->setStyleSheet(menu_style);
    }
}

void MainWindow::updateModeIndicator_(bool auto_mode) {
    if (!status_mode_) return;

    const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();
    if (auto_mode) {
        status_mode_->setText("Mode: Auto");
        status_mode_->setStyleSheet(
            QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
            .arg(spec.accent.name()));
    } else {
        status_mode_->setText("Mode: Manual");
        status_mode_->setStyleSheet(
            QString("color: %1 !important; font-weight: 600 !important; background: transparent !important;")
            .arg(spec.text_muted.name()));
    }
}

void MainWindow::updateStatusLabels_() {
    const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();

    const QColor color_ok = spec.accent2;
    const QColor color_bad("#FF6B6B");
    const QColor color_warning("#FFA500");
    const QColor color_muted = spec.text_muted;

    if (overall_connected_) {
        status_connection_->setText("Connected");
        status_connection_->setStyleSheet(
            QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
            .arg(color_ok.name()));
    } else {
        status_connection_->setText("Disconnected");
        status_connection_->setStyleSheet(
            QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
            .arg(color_bad.name()));
    }

    bool real_live = false;
    if (!seen_real_) {
        status_real_->setText("REAL: --");
        status_real_->setStyleSheet(
            QString("color: %1 !important; background: transparent !important;").arg(color_muted.name()));
    } else {
        const qint64 ms = real_timer_.elapsed();
        if (ms <= kStreamStaleMs) {
            status_real_->setText("REAL: LIVE");
            status_real_->setStyleSheet(
                QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
                .arg(color_ok.name()));
            real_live = true;
        } else {
            status_real_->setText("REAL: STALE");
            status_real_->setStyleSheet(
                QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
                .arg(color_warning.name()));
        }
    }

    bool sim_live = false;
    if (!seen_sim_) {
        status_sim_->setText("SIM: --");
        status_sim_->setStyleSheet(
            QString("color: %1 !important; background: transparent !important;").arg(color_muted.name()));
    } else {
        const qint64 ms = sim_timer_.elapsed();
        if (ms <= kStreamStaleMs) {
            status_sim_->setText("SIM: LIVE");
            status_sim_->setStyleSheet(
                QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
                .arg(color_ok.name()));
            sim_live = true;
        } else {
            status_sim_->setText("SIM: STALE");
            status_sim_->setStyleSheet(
                QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
                .arg(color_warning.name()));
        }
    }

    if (dashboard_) {
        dashboard_->setStreamLive(davinci_arm::models::Domain::Real, real_live);
        dashboard_->setStreamLive(davinci_arm::models::Domain::Sim, sim_live);
    }
    if (control_) {
        control_->setStreamLive(davinci_arm::models::Domain::Real, real_live);
        control_->setStreamLive(davinci_arm::models::Domain::Sim, sim_live);
    }
    if (calibration_) {
        calibration_->setStreamLive(davinci_arm::models::Domain::Real, real_live);
        calibration_->setStreamLive(davinci_arm::models::Domain::Sim, sim_live);
    }
}

void MainWindow::onThemeChanged_(davinci_arm::models::ThemeId id) {
    Q_UNUSED(id);

    QTimer::singleShot(10, this, [this]() {
        const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();

        const QString app_style = qApp->styleSheet();
        const QString custom_overrides = QString(
                                             "QToolBar#topbar { "
                                             "background-color: %1 !important; "
                                             "border: none !important; "
                                             "border-bottom: 1px solid %2 !important; "
                                             "padding: 0px !important; "
                                             "spacing: 0px !important; "
                                             "min-height: 40px !important; "
                                             "} "
                                             "QStatusBar#statusBar { "
                                             "background-color: %1 !important; "
                                             "color: %3 !important; "
                                             "border-top: 1px solid %2 !important; "
                                             "padding: 7px 20px !important; "
                                             "min-height: 20px !important; "
                                             "} ")
                                         .arg(spec.panel.name())
                                         .arg(spec.accent.name())
                                         .arg(spec.text.name());

        qApp->setStyleSheet(app_style + "\n" + custom_overrides);

        updateTopBarStyle_();
        updateStatusBarStyle_();
        updateStatusLabels_();

        if (topbar_) topbar_->update();
        if (top_tabs_) top_tabs_->update();
        if (statusBar()) statusBar()->update();
    });
}

void MainWindow::onNavChanged_(int row) {
    if (!stack_) return;

    row = std::clamp(row, 0, stack_->count() - 1);
    stack_->setCurrentIndex(row);

    if (top_tabs_ && top_tabs_->currentIndex() != row) {
        top_tabs_->setCurrentIndex(row);
    }

    if (top_tabs_) {
        top_tabs_->update();
        top_tabs_->repaint();
    }
}

void MainWindow::setTelemetryStore(davinci_arm::models::TelemetryStore* store) {
    store_ = store;
    if (dashboard_) dashboard_->setTelemetryStore(store_);
}

void MainWindow::setRecorderService(davinci_arm::core::services::RecorderService* recorder) {
    recorder_ = recorder;
    if (control_) control_->setRecorderService(recorder_);
}

void MainWindow::setConnected(bool connected) {
    overall_connected_ = connected;
    updateStatusLabels_();
}

void MainWindow::setControlMode(const QString& mode) {
    const QString normalized = mode.trimmed().toLower();
    if (normalized == "auto") {
        updateModeIndicator_(true);
        return;
    }
    if (normalized == "manual") {
        updateModeIndicator_(false);
        return;
    }

    if (status_mode_) {
        status_mode_->setText(QString("Mode: %1").arg(mode));
    }
}

void MainWindow::setSystemStatus(const QString& status) {
    Q_UNUSED(status);
}

void MainWindow::onTelemetry(const davinci_arm::models::TelemetrySample& sample) {
    if (dashboard_) dashboard_->onTelemetry(sample);
    if (control_) control_->onTelemetry(sample);
    if (calibration_) calibration_->onTelemetry(sample);

    if (sample.domain == Domain::Real) {
        if (!seen_real_) {
            seen_real_ = true;
            real_timer_.start();
        } else {
            real_timer_.restart();
        }
    } else if (sample.domain == Domain::Sim) {
        if (!seen_sim_) {
            seen_sim_ = true;
            sim_timer_.start();
        } else {
            sim_timer_.restart();
        }
    }
}

void MainWindow::showEvent(QShowEvent* event) {
    QMainWindow::showEvent(event);

    if (!initial_geometry_applied_) {
        initial_geometry_applied_ = true;
        applyResponsiveInitialGeometry_();
    }
}

void MainWindow::applyResponsiveInitialGeometry_() {
    QScreen* screen = nullptr;
    if (windowHandle() && windowHandle()->screen()) {
        screen = windowHandle()->screen();
    } else {
        screen = QGuiApplication::screenAt(QCursor::pos());
        if (!screen) screen = QGuiApplication::primaryScreen();
    }
    if (!screen) return;

    const QRect available = screen->availableGeometry();
    const int target_w = static_cast<int>(available.width() * 0.92);
    const int target_h = static_cast<int>(available.height() * 0.92);
    const QSize min_size = minimumSize();
    const int w = std::max(min_size.width(), std::min(target_w, available.width()));
    const int h = std::max(min_size.height(), std::min(target_h, available.height()));
    const int x = available.x() + (available.width() - w) / 2;
    const int y = available.y() + (available.height() - h) / 2;

    setGeometry(QRect(QPoint(x, y), QSize(w, h)));
}

void MainWindow::clampToCurrentScreen_() {
    QScreen* screen = nullptr;
    if (windowHandle() && windowHandle()->screen()) {
        screen = windowHandle()->screen();
    } else {
        screen = QGuiApplication::primaryScreen();
    }
    if (!screen) return;

    const QRect available = screen->availableGeometry();
    QRect geometry_rect = geometry();
    const QSize min_size = minimumSize();
    const int w = std::max(min_size.width(), std::min(geometry_rect.width(), available.width()));
    const int h = std::max(min_size.height(), std::min(geometry_rect.height(), available.height()));
    geometry_rect.setSize(QSize(w, h));

    if (geometry_rect.left() < available.left()) geometry_rect.moveLeft(available.left());
    if (geometry_rect.top() < available.top()) geometry_rect.moveTop(available.top());
    if (geometry_rect.right() > available.right()) geometry_rect.moveRight(available.right());
    if (geometry_rect.bottom() > available.bottom()) geometry_rect.moveBottom(available.bottom());

    setGeometry(geometry_rect);
}

void MainWindow::setupScreenTracking_() {
    QTimer::singleShot(0, this, [this]() {
        QWindow* window = windowHandle();
        if (!window) return;

        auto attach_available_geometry_listener = [this](QScreen* screen) {
            if (!screen) return;
            if (avail_geom_conn_) {
                QObject::disconnect(avail_geom_conn_);
                avail_geom_conn_ = {};
            }
            avail_geom_conn_ = connect(screen,
                                       &QScreen::availableGeometryChanged,
                                       this,
            [this](const QRect&) {
                clampToCurrentScreen_();
            });
        };

        attach_available_geometry_listener(window->screen());

        connect(window,
                &QWindow::screenChanged,
                this,
        [this, attach_available_geometry_listener](QScreen* screen) mutable {
            clampToCurrentScreen_();
            attach_available_geometry_listener(screen);
        });
    });
}

void MainWindow::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits) {
    limits_ = limits;

    if (dashboard_) dashboard_->setLimitsRegistry(limits_);
    if (control_) control_->setLimitsRegistry(limits_);
    if (calibration_) calibration_->setLimitsRegistry(limits_);
}

void MainWindow::setCalibrationService(davinci_arm::services::CalibrationService* service) {
    calibration_service_ = service;
    if (calibration_) calibration_->setCalibrationService(service);
}

void MainWindow::setStreamLive(davinci_arm::models::Domain domain, bool live) {
    if (dashboard_) dashboard_->setStreamLive(domain, live);
    if (control_) control_->setStreamLive(domain, live);
    if (calibration_) calibration_->setStreamLive(domain, live);
}

}  // namespace davinci_arm::app
