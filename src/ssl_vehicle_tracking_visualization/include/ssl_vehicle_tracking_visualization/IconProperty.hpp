#pragma once

#include <rviz_common/properties/property.hpp>

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
class IconProperty : public rviz_common::properties::Property {
 private:
  QIcon icon_;

 public :
  /**
   * @brief Just execute super constructor. Nothing done here.
   * @see Property::Property
   */
  explicit IconProperty(const QString &name = QString(),
                        QVariant default_value = QVariant(),
                        const QString &description = QString(),
                        Property *parent = nullptr,
                        const char *changed_slot = nullptr,
                        QObject *receiver = nullptr);

  [[nodiscard]] QVariant getViewData(int column, int role) const override;
  void setIcon(const QIcon &icon) override;
};

}  // namespace rviz
