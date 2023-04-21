#include "ssl_vehicle_tracking_visualization/IconProperty.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QStyleOptionViewItem>

#include <cmath>
#include <utility>

namespace rviz_common::properties {

IconProperty::IconProperty(const QString &name,
                           QVariant default_value,
                           const QString &description,
                           rviz_common::properties::Property *parent,
                           const char *changed_slot,
                           QObject *receiver) : Property(name,
                                                         std::move(default_value),
                                                         description,
                                                         parent,
                                                         changed_slot,
                                                         receiver) {

}

QVariant IconProperty::getViewData(int column, int role) const {
  return (column == 0 && role == Qt::DecorationRole) ? icon_ : Property::getViewData(column, role);
}

void IconProperty::setIcon(const QIcon &icon) {
  icon_ = icon;
}

}
