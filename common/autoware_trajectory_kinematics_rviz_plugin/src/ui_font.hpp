// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UI_FONT_HPP_
#define UI_FONT_HPP_

#include <QFont>
#include <QFontDatabase>
#include <QStringList>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{

/// @brief Returns a cached UI font: first match among preferred sans families, 13px, demibold, full
/// hinting.
inline QFont makePanelUiFont()
{
  static const QFont kFont = []() {
    static const QStringList kPreferredFamilies = {
      QStringLiteral("Segoe UI"),        QStringLiteral("Ubuntu"),      QStringLiteral("Noto Sans"),
      QStringLiteral("Liberation Sans"), QStringLiteral("DejaVu Sans"),
    };

    const QStringList families = QFontDatabase().families();
    QString chosen;
    for (const QString & fam : kPreferredFamilies) {
      if (families.contains(fam)) {
        chosen = fam;
        break;
      }
    }

    QFont font;
    if (!chosen.isEmpty()) {
      font.setFamily(chosen);
    }
    font.setPixelSize(13);
    font.setWeight(QFont::DemiBold);
    font.setStyleHint(QFont::SansSerif, QFont::PreferAntialias);
    font.setHintingPreference(QFont::PreferFullHinting);
    return font;
  }();
  return kFont;
}

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

#endif  // UI_FONT_HPP_
