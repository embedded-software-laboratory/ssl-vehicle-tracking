#pragma once

#include <rviz_common/properties/bool_property.hpp>

/*
 * Move to rviz namespace to match property naming convention.
 */
namespace rviz_common::properties {

/**
 * @author Simon Schaefer
 * @date 01.01.2019
 * @brief Online property with an color indicator display method.
 * @details This property will draw an online/offline indicator together with an descriptive text.
 */
class OnlineProperty : public rviz_common::properties::BoolProperty {
 private:
  QMap<QString, QColor> color_mapping_;
  QMap<int, QString> message_mapping_;
  int mode_ = 0;
 public :
  /**
   * @brief Just execute super constructor. Nothing done here.
   * @see EnumProperty::EnumProperty
   */
  explicit OnlineProperty(const QString &name = QString(),
                          bool default_value = false,
                          const QString &description = QString(),
                          Property *parent = nullptr,
                          const char *changed_slot = nullptr,
                          QObject *receiver = nullptr);

  /**
   * @brief Will draw the online/offline indicator.
   * @details Will draw an circle with a fill corresponding to online/offline status.
   * @return always true, this is the way the system know to repaint.
   */
  bool paint(QPainter *painter, const QStyleOptionViewItem &option) const override;

  int mode() const;
  void setMode(int mode);
};

}  // namespace rviz
