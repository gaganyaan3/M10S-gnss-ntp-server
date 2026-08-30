#include "CalibratedFqMap.h"

namespace {
struct CalibratedFqPoint {
  int16_t temperatureDeciC;
  uint32_t ticksPerSecond;
};

constexpr CalibratedFqPoint PPS_MCPWM_TICKS_PER_SECOND_MAP[] = {
    {300, 79998240UL},
    {306, 79998232UL},
    {311, 79998219UL},
    {317, 79998205UL},
    {322, 79998193UL},
    {328, 79998175UL},
    {333, 79998161UL},
    {339, 79998145UL},
    {344, 79998126UL},
    {350, 79998111UL},
    {356, 79998086UL},
    {361, 79998068UL},
    {367, 79998050UL},
    {372, 79998019UL},
    {378, 79997997UL},
    {383, 79997973UL},
    {389, 79997946UL},
    {394, 79997923UL},
    {400, 79997899UL},
    {406, 79997872UL},
    {411, 79997850UL},
    {417, 79997832UL},
    {422, 79997807UL},
    {433, 79997746UL},
    {456, 79997674UL},
    {517, 79998085UL},
    {522, 79998083UL},
    {528, 79998080UL},
    {533, 79998074UL},
    {539, 79998068UL},
    {544, 79998062UL},
    {550, 79998058UL},
    {556, 79998050UL},
    {561, 79998046UL},
    {567, 79998043UL},
    {572, 79998040UL},
    {578, 79998038UL},
    {583, 79998037UL},
    {589, 79998038UL},
    {594, 79998038UL},
    {600, 79998040UL},
    {606, 79998044UL},
    {611, 79998046UL},
    {617, 79998046UL},
    {622, 79998050UL},
    {628, 79998050UL},
    {633, 79998051UL},
    {639, 79998055UL},
    {644, 79998062UL},
    {650, 79998070UL},
    {656, 79998081UL},
    {661, 79998085UL},
    {667, 79998099UL},
    {678, 79998129UL},
    {683, 79998138UL},
    {689, 79998154UL},
    {694, 79998167UL},
    {700, 79998184UL},
    {706, 79998204UL},
    {711, 79998223UL},
    {717, 79998237UL},
    {722, 79998248UL},
};
}  // namespace

CalibratedFqValue CalibratedFqMap::nearestForTemperatureC(float temperatureC) {
  const int16_t temperatureDeciC = temperatureToDeciC(temperatureC);
  const CalibratedFqPoint* nearest = &PPS_MCPWM_TICKS_PER_SECOND_MAP[0];
  uint16_t nearestDiff = abs(temperatureDeciC - nearest->temperatureDeciC);

  for (const CalibratedFqPoint& point : PPS_MCPWM_TICKS_PER_SECOND_MAP) {
    const uint16_t diff = abs(temperatureDeciC - point.temperatureDeciC);
    if (diff < nearestDiff) {
      nearest = &point;
      nearestDiff = diff;
    }
  }

  return {nearest->temperatureDeciC, nearest->ticksPerSecond};
}

int16_t CalibratedFqMap::temperatureToDeciC(float temperatureC) {
  return static_cast<int16_t>(temperatureC * 10.0f + (temperatureC >= 0.0f ? 0.5f : -0.5f));
}
