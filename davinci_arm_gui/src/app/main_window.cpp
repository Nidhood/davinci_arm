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
#include <QStatusBar>
#include <QStyle>
#include <QTimer>
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
    : QMainWindow(parent)
{
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

    connect(
        &davinci_arm::ui::style::ThemeManager::instance(),
        &davinci_arm::ui::style::ThemeManager::themeChanged,
        this,
        &MainWindow::onThemeChanged_);

    if (top_tabs_) {
        top_tabs_->setCurrentIndex(0);
    }
    onNavChanged_(0);
}

void MainWindow::buildUi_()
{
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

void MainWindow::buildTopBar_()
{
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

    auto add_theme = [&](const QString& name, davinci_arm::models::ThemeId id) {
        QAction* action = theme_menu->addAction(name);
        action->setData(static_cast<int>(id));
        connect(action, &QAction::triggered, this, &MainWindow::onThemeAction_);
    };

    add_theme("Light", davinci_arm::models::ThemeId::Light);
    add_theme("Dark", davinci_arm::models::ThemeId::Dark);
    add_theme("MATLAB", davinci_arm::models::ThemeId::MATLAB);
    add_theme("Tron Evolution", davinci_arm::models::ThemeId::TronEvolution);
    add_theme("Tron Ares", davinci_arm::models::ThemeId::TronAres);
    add_theme("Cyberpunk Neon", davinci_arm::models::ThemeId::CyberpunkNeon);

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
    top_tabs_->addTab("Drawing App");

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

void MainWindow::onThemeAction_()
{
    auto* action = qobject_cast<QAction*>(sender());
    if (!action) {
        return;
    }

    const auto id = static_cast<davinci_arm::models::ThemeId>(action->data().toInt());
    davinci_arm::ui::style::ThemeManager::instance().apply(id);
}

void MainWindow::connectPageSignals_()
{
    connect(control_, &davinci_arm::ui::pages::ControlPanelPage::refAngleChanged,
            this, &MainWindow::refAngleChanged);
    connect(control_, &davinci_arm::ui::pages::ControlPanelPage::pwmChanged,
            this, &MainWindow::pwmChanged);
    connect(control_, &davinci_arm::ui::pages::ControlPanelPage::autoModeChanged,
    this, [this](bool auto_mode) {
        updateModeIndicator_(auto_mode);
        emit autoModeChanged(auto_mode);
    });
    connect(control_, &davinci_arm::ui::pages::ControlPanelPage::stopRequested,
            this, &MainWindow::stopRequested);
    connect(control_, &davinci_arm::ui::pages::ControlPanelPage::jointBatchCommandRequested,
            this, &MainWindow::jointBatchCommandRequested);
}

void MainWindow::setupStatusBar_()
{
    status_connection_ = new QLabel("Disconnected");
    status_connection_->setObjectName("statusLabel");

    status_mode_ = new QLabel("Mode: Manual");
    status_mode_->setObjectName("statusLabel");

    status_real_ = new QLabel("REAL: --");
    status_real_->setObjectName("statusLabel");

    status_sim_ = new QLabel("SIM: --");
    status_sim_->setObjectName("statusLabel");

    auto create_separator = [this]() {
        auto* sep = new QLabel(" | ");
        sep->setObjectName("statusSeparator");
        return sep;
    };

    statusBar()->setObjectName("statusBar");
    statusBar()->setSizeGripEnabled(false);
    statusBar()->addPermanentWidget(status_connection_);
    statusBar()->addPermanentWidget(create_separator());
    statusBar()->addPermanentWidget(status_mode_);
    statusBar()->addPermanentWidget(create_separator());
    statusBar()->addPermanentWidget(status_real_);
    statusBar()->addPermanentWidget(create_separator());
    statusBar()->addPermanentWidget(status_sim_);

    updateStatusBarStyle_();

    status_timer_ = new QTimer(this);
    status_timer_->setInterval(200);
    status_timer_->setTimerType(Qt::CoarseTimer);
    connect(status_timer_, &QTimer::timeout, this, &MainWindow::updateStatusLabels_);
    status_timer_->start();
}

void MainWindow::updateStatusBarStyle_()
{
    const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();

    QString statusBarStyle = QString(
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

    statusBar()->setStyleSheet(statusBarStyle);
}

void MainWindow::updateTopBarStyle_()
{
    const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();

    QString topBarStyle = QString(
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

    topbar_->setStyleSheet(topBarStyle);

    QString tabBarStyle = QString(
                              "QTabBar#topTabs { "
                              "background: transparent !important; "
                              "border: none !important; "
                              "} "
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

    top_tabs_->setStyleSheet(tabBarStyle);

    QString menuStyle = QString(
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
        theme_btn_->menu()->setStyleSheet(menuStyle);
    }
}

void MainWindow::updateModeIndicator_(bool auto_mode)
{
    if (!status_mode_) {
        return;
    }

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

void MainWindow::updateStatusLabels_()
{
    const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();
    const QColor colorOk = spec.accent2;
    const QColor colorBad = QColor("#FF6B6B");
    const QColor colorWarning = QColor("#FFA500");
    const QColor colorMuted = spec.text_muted;

    if (overall_connected_) {
        status_connection_->setText("Connected");
        status_connection_->setStyleSheet(
            QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
            .arg(colorOk.name()));
    } else {
        status_connection_->setText("Disconnected");
        status_connection_->setStyleSheet(
            QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
            .arg(colorBad.name()));
    }

    bool real_live = false;
    if (!seen_real_) {
        status_real_->setText("REAL: --");
        status_real_->setStyleSheet(QString("color: %1 !important; background: transparent !important;")
                                    .arg(colorMuted.name()));
    } else {
        const qint64 ms = real_timer_.elapsed();
        if (ms <= kStreamStaleMs) {
            status_real_->setText("REAL: LIVE");
            status_real_->setStyleSheet(
                QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
                .arg(colorOk.name()));
            real_live = true;
        } else {
            status_real_->setText("REAL: STALE");
            status_real_->setStyleSheet(
                QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
                .arg(colorWarning.name()));
        }
    }

    bool sim_live = false;
    if (!seen_sim_) {
        status_sim_->setText("SIM: --");
        status_sim_->setStyleSheet(QString("color: %1 !important; background: transparent !important;")
                                   .arg(colorMuted.name()));
    } else {
        const qint64 ms = sim_timer_.elapsed();
        if (ms <= kStreamStaleMs) {
            status_sim_->setText("SIM: LIVE");
            status_sim_->setStyleSheet(
                QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
                .arg(colorOk.name()));
            sim_live = true;
        } else {
            status_sim_->setText("SIM: STALE");
            status_sim_->setStyleSheet(
                QString("color: %1 !important; font-weight: bold !important; background: transparent !important;")
                .arg(colorWarning.name()));
        }
    }

    if (dashboard_) {
        dashboard_->setStreamLive(Domain::Real, real_live);
        dashboard_->setStreamLive(Domain::Sim, sim_live);
    }
    if (control_) {
        control_->setStreamLive(Domain::Real, real_live);
        control_->setStreamLive(Domain::Sim, sim_live);
    }
    if (calibration_) {
        calibration_->setStreamLive(Domain::Real, real_live);
        calibration_->setStreamLive(Domain::Sim, sim_live);
    }
}

void MainWindow::onThemeChanged_(davinci_arm::models::ThemeId id)
{
    Q_UNUSED(id);

    QTimer::singleShot(10, this, [this]() {
        const auto& spec = davinci_arm::ui::style::ThemeManager::instance().currentSpec();

        QString appStyle = qApp->styleSheet();
        QString customOverrides = QString(
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

        qApp->setStyleSheet(appStyle + "\n" + customOverrides);

        updateTopBarStyle_();
        updateStatusBarStyle_();
        updateStatusLabels_();

        if (topbar_) {
            topbar_->update();
        }
        if (top_tabs_) {
            top_tabs_->update();
        }
        if (statusBar()) {
            statusBar()->update();
        }
    });
}

void MainWindow::onNavChanged_(int row)
{
    if (!stack_) {
        return;
    }

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

void MainWindow::setTelemetryStore(davinci_arm::models::TelemetryStore* store)
{
    store_ = store;
    if (dashboard_) {
        dashboard_->setTelemetryStore(store_);
    }
}

void MainWindow::setRecorderService(davinci_arm::core::services::RecorderService* recorder)
{
    recorder_ = recorder;
    if (control_) {
        control_->setRecorderService(recorder_);
    }
}

void MainWindow::setConnected(bool connected)
{
    overall_connected_ = connected;
    updateStatusLabels_();
}

void MainWindow::setControlMode(const QString& mode)
{
    status_mode_->setText(QString("Mode: %1").arg(mode));
}

void MainWindow::setSystemStatus(const QString& status)
{
    Q_UNUSED(status);
}

void MainWindow::onTelemetry(const davinci_arm::models::TelemetrySample& sample)
{
    if (dashboard_) {
        dashboard_->onTelemetry(sample);
    }
    if (control_) {
        control_->onTelemetry(sample);
    }
    if (calibration_) {
        calibration_->onTelemetry(sample);
    }

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

void MainWindow::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);

    if (!initial_geometry_applied_) {
        initial_geometry_applied_ = true;
        applyResponsiveInitialGeometry_();
    }
}

void MainWindow::applyResponsiveInitialGeometry_()
{
    QScreen* screen = nullptr;
    if (windowHandle() && windowHandle()->screen()) {
        screen = windowHandle()->screen();
    } else {
        screen = QGuiApplication::screenAt(QCursor::pos());
        if (!screen) {
            screen = QGuiApplication::primaryScreen();
        }
    }

    if (!screen) {
        return;
    }

    const QRect avail = screen->availableGeometry();
    const int targetW = static_cast<int>(avail.width() * 0.92);
    const int targetH = static_cast<int>(avail.height() * 0.92);

    const QSize minSz = minimumSize();
    const int w = std::max(minSz.width(), std::min(targetW, avail.width()));
    const int h = std::max(minSz.height(), std::min(targetH, avail.height()));

    const int x = avail.x() + (avail.width() - w) / 2;
    const int y = avail.y() + (avail.height() - h) / 2;
    setGeometry(QRect(QPoint(x, y), QSize(w, h)));
}

void MainWindow::clampToCurrentScreen_()
{
    QScreen* screen = nullptr;
    if (windowHandle() && windowHandle()->screen()) {
        screen = windowHandle()->screen();
    } else {
        screen = QGuiApplication::primaryScreen();
    }

    if (!screen) {
        return;
    }

    const QRect avail = screen->availableGeometry();
    QRect g = geometry();

    const QSize minSz = minimumSize();
    const int w = std::max(minSz.width(), std::min(g.width(), avail.width()));
    const int h = std::max(minSz.height(), std::min(g.height(), avail.height()));
    g.setSize(QSize(w, h));

    if (g.left() < avail.left()) {
        g.moveLeft(avail.left());
    }
    if (g.top() < avail.top()) {
        g.moveTop(avail.top());
    }
    if (g.right() > avail.right()) {
        g.moveRight(avail.right());
    }
    if (g.bottom() > avail.bottom()) {
        g.moveBottom(avail.bottom());
    }

    setGeometry(g);
}

void MainWindow::setupScreenTracking_()
{
    QTimer::singleShot(0, this, [this]() {
        QWindow* w = windowHandle();
        if (!w) {
            return;
        }

        auto attachListener = [this](QScreen* s) {
            if (!s) {
                return;
            }
            if (avail_geom_conn_) {
                QObject::disconnect(avail_geom_conn_);
                avail_geom_conn_ = {};
            }
            avail_geom_conn_ = connect(
                                   s,
                                   &QScreen::availableGeometryChanged,
                                   this,
            [this](const QRect&) {
                clampToCurrentScreen_();
            });
        };

        attachListener(w->screen());

        connect(w, &QWindow::screenChanged, this, [this, attachListener](QScreen* s) mutable {
            clampToCurrentScreen_();
            attachListener(s);
        });
    });
}

void MainWindow::setLimitsRegistry(const davinci_arm::infra::ros::LimitsRegistry* limits)
{
    limits_ = limits;

    if (dashboard_) {
        dashboard_->setLimitsRegistry(limits);
    }
    if (control_) {
        control_->setLimitsRegistry(limits);
    }
    if (calibration_) {
        calibration_->setLimitsRegistry(limits);
    }
}

void MainWindow::setCalibrationService(davinci_arm::services::CalibrationService* service)
{
    calibration_service_ = service;
    if (calibration_) {
        calibration_->setCalibrationService(service);
    }
}

void MainWindow::setStreamLive(davinci_arm::models::Domain domain, bool live)
{
    if (dashboard_) {
        dashboard_->setStreamLive(domain, live);
    }
    if (control_) {
        control_->setStreamLive(domain, live);
    }
    if (calibration_) {
        calibration_->setStreamLive(domain, live);
    }
}

}  // namespace davinci_arm::app
