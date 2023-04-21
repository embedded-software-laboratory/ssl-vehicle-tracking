#include "ssl_vehicle_tracking_visualization/OnlineProperty.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QStyleOptionViewItem>

#include <cmath>

namespace rviz_common::properties {

OnlineProperty::OnlineProperty(const QString &name,
                               bool default_value,
                               const QString &description,
                               rviz_common::properties::Property *parent,
                               const char *changed_slot,
                               QObject *receiver) : BoolProperty(name,
                                                                 default_value,
                                                                 description,
                                                                 parent,
                                                                 changed_slot,
                                                                 receiver) {

  message_mapping_[0] = "Offline";
  color_mapping_["Offline"] = Qt::red;

  message_mapping_[1] = "Intermediate";
  color_mapping_["Intermediate"] = Qt::darkYellow;

  message_mapping_[2] = "Online";
  color_mapping_["Online"] = Qt::green;
}

bool OnlineProperty::paint(QPainter *painter, const QStyleOptionViewItem &option) const {
  // Save all settings for later restoration
  painter->save();
  // Sett high quality render
  painter->setRenderHint(QPainter::Antialiasing);

  int value = this->mode_;
  QString text = message_mapping_[value];
  QColor color = color_mapping_[text];
  QRectF rect = option.rect;
  double line_length = rect.height();
  rect.adjust(rect.height() + 4, 1, 0, 0);
  painter->drawText(rect, text);
  rect.adjust(-rect.height() - 4, -1, 0, 0);

  QPointF center = rect.bottomLeft();
  center += {rect.height() / 2.0f + 2, -rect.height() / 2.0f};

  painter->setBrush(QBrush(color));
  painter->drawEllipse(center, rect.height() / 2.0f - 2, rect.height() / 2.0f - 2);

  // Restore whatever was set before
  painter->restore();
  return true;
}
int OnlineProperty::mode() const {
  return mode_;
}
void OnlineProperty::setMode(int mode) {
  mode_ = mode;
}
}
