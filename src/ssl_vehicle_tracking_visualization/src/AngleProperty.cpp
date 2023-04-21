#include "ssl_vehicle_tracking_visualization/AngleProperty.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QStyleOptionViewItem>

#include <cmath>

namespace rviz_common::properties {

AngleProperty::AngleProperty(const QString &name,
                             float default_value,
                             const QString &description,
                             rviz_common::properties::Property *parent,
                             const char *changed_slot,
                             QObject *receiver) : FloatProperty(name,
                                                                default_value,
                                                                description,
                                                                parent,
                                                                changed_slot,
                                                                receiver) {

}

bool AngleProperty::paint(QPainter *painter, const QStyleOptionViewItem &option) const {

  // Save all settings for later restoration
  painter->save();
  // Sett high quality render
  painter->setRenderHint(QPainter::Antialiasing);
  if (!(getViewFlags(0) & Qt::ItemIsEnabled)) {
    painter->setPen(QColor(Qt::lightGray));
  }
  QRectF rect = option.rect;
  auto angle_in_rad = static_cast<float>(value_.toFloat());
  double line_length = rect.height();
  auto angle_in_deg_parts = static_cast<float>(angle_in_rad * 180.0 / M_PI);
  QString text = QString::number(angle_in_deg_parts, 'f', 2) +
      ""; // https://doc.qt.io/qt-5/qstring.html#argument-formats
  rect.adjust(rect.height() + 4, 1, 0, 0);
  painter->drawText(rect, text);
  rect.adjust(-rect.height() - 4, -1, 0, 0);

  QPointF starting_point;
  starting_point.setX(
      rect.x() + rect.height() / 2.0f - line_length / 4.0f * std::cos(-angle_in_rad));
  starting_point.setY(
      rect.y() + rect.height() / 2.0f - line_length / 4.0f * std::sin(-angle_in_rad));

  QPointF starting_point_r;
  starting_point_r.setX(
      rect.x() + rect.height() / 2.0f - line_length / 2.0f * std::cos(-angle_in_rad + 0.5f));
  starting_point_r.setY(
      rect.y() + rect.height() / 2.0f - line_length / 2.0f * std::sin(-angle_in_rad + 0.5f));

  QPointF starting_point_l;
  starting_point_l.setX(
      rect.x() + rect.height() / 2.0f - line_length / 2.0f * std::cos(-angle_in_rad - 0.5f));
  starting_point_l.setY(
      rect.y() + rect.height() / 2.0f - line_length / 2.0f * std::sin(-angle_in_rad - 0.5f));

  QPointF end_point;
  end_point.setX(rect.x() + rect.height() / 2.0f + line_length / 2.0f * std::cos(-angle_in_rad));
  end_point.setY(rect.y() + rect.height() / 2.0f + line_length / 2.0f * std::sin(-angle_in_rad));

  QPolygonF polyline;
  polyline << end_point;
  polyline << starting_point_r;
  polyline << starting_point;
  polyline << starting_point_l;
  polyline << end_point;
  painter->drawPolygon(polyline);

  // Restore whatever was set before
  painter->restore();
  return true;
}
}
