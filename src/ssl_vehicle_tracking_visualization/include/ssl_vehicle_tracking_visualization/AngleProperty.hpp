#pragma once

#include <rviz_common/properties/float_property.hpp>

/*
 * Move to rviz namespace to match property naming convention.
 */
namespace rviz_common::properties {

/**
 * @author Simon Schaefer
 * @date 01.01.2019
 * @brief Simple Float property with an advanced display method.
 * @details This property will draw an arrow next to the value. The value is represented in fractions of Pi.
 */
class AngleProperty : public rviz_common::properties::FloatProperty {
 public :
  /**
   * @brief Just execute super constructor. Nothing done here.
   * @see FloatProperty::FloatProperty
   */
  explicit AngleProperty(const QString &name = QString(),
                         float default_value = 0,
                         const QString &description = QString(),
                         Property *parent = nullptr,
                         const char *changed_slot = nullptr,
                         QObject *receiver = nullptr);

  /**
   * @brief Will draw a new representation of an angle.
   * @details Will draw an arrow with zero at top and counterclockwise positive.
   * @return always true, this is the way the system know to repaint.
   */
  bool paint(QPainter *painter, const QStyleOptionViewItem &option) const override;
};

}  // namespace rviz
