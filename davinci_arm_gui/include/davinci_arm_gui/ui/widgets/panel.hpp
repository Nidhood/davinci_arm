#pragma once

#include <QFrame>
#include <QString>

class QLabel;
class QVBoxLayout;

namespace davinci_arm::ui::widgets {

class Panel final : public QFrame {
    Q_OBJECT

public:
    explicit Panel(const QString& title = {}, QWidget* parent = nullptr);

    [[nodiscard]] QVBoxLayout* bodyLayout() const noexcept {
        return body_;
    }

    void setTitle(const QString& title);
    [[nodiscard]] QString title() const;

private:
    QLabel* title_{nullptr};
    QVBoxLayout* root_{nullptr};
    QVBoxLayout* body_{nullptr};
};

}  // namespace davinci_arm::ui::widgets
