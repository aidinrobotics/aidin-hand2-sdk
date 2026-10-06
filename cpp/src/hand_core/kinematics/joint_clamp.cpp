// JointPositionCommand::clamp, pulling a target into each finger's reachable workspace
//
// The parallel wrist couples abduction and flexion, so the measured boundary is a chain of
// line and arc pieces. Inside is left alone, outside is projected to the nearest point
// Past the last knot the boundary is a zero-abduction floor up to the flexion limit
// Abduction is symmetric, handled as an absolute value with the sign put back
// Thumb j0 and j3 get a constant limit, and so does the long j3 at its top
// The long j3 floor is a closed form of the clamped pair, the formula docs 14 section 3.2 gives
//
// Slots: long finger abduction 0, flexion 1, thumb abduction 2, flexion 1
// `ctest -R joint_clamp` checks the seams and the projection after a table change

#include <aidin_hand2/types/command.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace aidin_hand2
{
namespace
{

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
// Slack for the inside test and the piece range comparison
constexpr double kToleranceDeg = 1e-9;

// One boundary piece, the two knots being its end points
//   Line  the segment between them, arc fields 0
//   Arc   y = arc_center_abduction_deg + sign(R)*sqrt(R^2 - (x - arc_center_flexion_deg)^2)
// The centre follows from the knots and R, and keeps ten digits: at four it misses a knot by up to
// 6e-4 deg, far more where an arc meets its knot near vertical, and a target on the boundary
// then reads as outside
enum class PieceKind { Line, Arc };
// The piece count is part of each table's type, so adding a piece means raising its count too
constexpr std::size_t kLongPieceCount  = 5;
constexpr std::size_t kThumbPieceCount = 4;

struct BoundaryPiece
{
  PieceKind kind;
  // Flexion and abduction of the two knots
  double flexion_lo_deg,   flexion_hi_deg;
  double abduction_lo_deg, abduction_hi_deg;

  // Arc only
  double arc_center_flexion_deg;
  double arc_center_abduction_deg;
  double arc_signed_radius_deg;
};

// Shared by index, middle, ring and baby, tuned on the 2026-10-01 right hand (hand type a)
constexpr std::array<BoundaryPiece, kLongPieceCount> kLongBoundary{{
    // kind            flexion lo    hi   abduction lo   hi    arc centre flexion  centre abduction  signed radius
    {PieceKind::Arc,    0.00,  11.56,    0.00,  30.00, -142.7639230149,   72.2389250017, -160.0},
    {PieceKind::Line,  11.56,  50.50,   30.00,  30.00,    0.0,             0.0,               0.0},
    {PieceKind::Arc,   50.50,  87.50,   30.00,  18.50,  104.1494008616,  137.3393766853, -120.0},
    {PieceKind::Arc,   87.50,  93.20,   18.50,  12.40,   76.0586998853,    2.0958343191,   20.0},
    {PieceKind::Arc,   93.20,  96.20,   12.40,   0.00,  121.1992042337,   12.6110977985,  -28.0},
}};
// Same tuning round, measured with the thumb j0 at 93 deg
// The last piece is all but vertical: flexion ends at 59 deg whatever the abduction
constexpr std::array<BoundaryPiece, kThumbPieceCount> kThumbBoundary{{
    {PieceKind::Arc,    0.00,   9.40,    0.00,  29.50, -423.8055121749,  151.2907394727, -450.0},
    {PieceKind::Arc,    9.40,  48.00,   29.50,  44.50,  -61.5421725476,  269.2231906892, -250.0},
    {PieceKind::Arc,   48.00,  58.40,   44.50,  39.43,   27.0303586384,  -11.7163156136,   60.0},
    {PieceKind::Line,  58.40,  59.00,   39.43,   0.00,    0.0,             0.0,               0.0},
}};

// The flexion limit is the last knot itself, so nothing is allowed past what was measured
// That leaves the zero-abduction floor with no length, and the knot as the projection candidate
constexpr double kLongFlexionMaxDeg  = 96.20;
constexpr double kThumbFlexionMaxDeg = 59.00;

// Upper limits in degrees for the joints outside the coupled pairs, the lower being 0
// Each is the stop found by bending that joint by hand, on the right hand the tables were tuned on
// The long j3 stop held within 0.1 deg at every abduction, so it is the joint's own stop and stays
// constant, where the floor below follows the pose
constexpr double kLongDistalMaxDeg = 80.0;   // long finger j3
constexpr double kThumbJ3MaxDeg    = 75.0;   // thumb j3
constexpr double kThumbJ0MaxDeg    = 108.8;  // thumb j0 CMC

// Long finger slots, as (abduction, flexion, j3)
struct LongDistal
{
  int abduction_slot;
  int flexion_slot;
  int distal_slot;
};
constexpr std::array<LongDistal, 4> kLongDistals{{
    {4, 5, 6}, {7, 8, 9}, {10, 11, 12}, {13, 14, 15},   // index, middle, ring, baby
}};

// Long j3 floor constants, a0..a10 and b0..b8 of docs 14 section 3.2, printed there to the same digits
// They fold the long finger linkage of hand type a, which type b shares, into ratios; the formula
// matches the angle at which the j3 actuator meets its home stop within 0.0004 deg
constexpr std::array<double, 11> kDistalFloorA{{
    0.819728, 1.79827, 0.287368, 0.142105, 0.371072, 0.34125, 0.16875, 1.02019, 0.0825804, 0.574736, 0.28421,
}};
constexpr std::array<double, 9> kDistalFloorB{{
    -3.26693, -2.91243, -1.56229, -2.13874, 2.10835, 3.93041, 11.923, -3.12458, -5.82487,
}};

// Clamps one finger's coupled pair, the table starting at flexion 0 with no gap between pieces
template <std::size_t PieceCount>
void clamp_finger_workspace(std::array<double, kActiveJointCount>& target_rad,
                            int abduction_slot, int flexion_slot,
                            const std::array<BoundaryPiece, PieceCount>& boundary,
                            double flexion_max_deg)
{
  const double flexion_deg = std::max(0.0, target_rad[flexion_slot] / kDeg2Rad);
  const double commanded_abduction_deg = target_rad[abduction_slot] / kDeg2Rad;
  // Symmetric, so the sign is put back at the end
  const double abduction_deg  = std::fabs(commanded_abduction_deg);
  const double abduction_sign = commanded_abduction_deg < 0.0 ? -1.0 : 1.0;

  // Past the last knot no piece matches, and the projection below takes over
  for (std::size_t index = 0; index < PieceCount; ++index) {
    const BoundaryPiece& piece = boundary[index];
    if (flexion_deg > piece.flexion_hi_deg + kToleranceDeg) continue;

    double abduction_limit_deg = 0.0;
    if (piece.kind == PieceKind::Line) {
      const double along_ratio = (flexion_deg - piece.flexion_lo_deg)
                               / (piece.flexion_hi_deg - piece.flexion_lo_deg);
      abduction_limit_deg = piece.abduction_lo_deg
                          + along_ratio * (piece.abduction_hi_deg - piece.abduction_lo_deg);
    } else {
      const double flexion_from_center = flexion_deg - piece.arc_center_flexion_deg;
      double under_root = piece.arc_signed_radius_deg * piece.arc_signed_radius_deg
                        - flexion_from_center * flexion_from_center;
      // Rounding at a piece end can give about -1e-10
      if (under_root < 0.0) under_root = 0.0;
      abduction_limit_deg = piece.arc_center_abduction_deg
          + (piece.arc_signed_radius_deg > 0.0 ? 1.0 : -1.0) * std::sqrt(under_root);
    }

    if (abduction_deg <= abduction_limit_deg + kToleranceDeg) {
      // The test above only reads flexion, so a negative one has to be pulled up here
      if (target_rad[flexion_slot] < 0.0) target_rad[flexion_slot] = 0.0;
      return;
    }
    // Only one piece can contain this flexion
    break;
  }

  // The floor comes first, and being horizontal it only clamps the flexion
  double nearest_flexion_deg = std::clamp(flexion_deg, boundary[PieceCount - 1].flexion_hi_deg,
                                          flexion_max_deg);
  double nearest_abduction_deg = 0.0;
  double nearest_distance = (flexion_deg - nearest_flexion_deg) * (flexion_deg - nearest_flexion_deg)
                          + abduction_deg * abduction_deg;

  for (std::size_t index = 0; index < PieceCount; ++index) {
    const BoundaryPiece& piece = boundary[index];
    double candidate_flexion_deg = 0.0;
    double candidate_abduction_deg = 0.0;

    if (piece.kind == PieceKind::Line) {
      // Point to segment projection: t = ((P-A).v)/(v.v) clamped to [0,1], then A + t*v
      const double along_flexion_deg   = piece.flexion_hi_deg   - piece.flexion_lo_deg;
      const double along_abduction_deg = piece.abduction_hi_deg - piece.abduction_lo_deg;
      const double dot_product = (flexion_deg   - piece.flexion_lo_deg)   * along_flexion_deg
                               + (abduction_deg - piece.abduction_lo_deg) * along_abduction_deg;
      const double along_length = along_flexion_deg * along_flexion_deg
                                + along_abduction_deg * along_abduction_deg;
      const double along_ratio = std::clamp(dot_product / along_length, 0.0, 1.0);
      candidate_flexion_deg   = piece.flexion_lo_deg   + along_ratio * along_flexion_deg;
      candidate_abduction_deg = piece.abduction_lo_deg + along_ratio * along_abduction_deg;
    } else {
      // Nearest point on a circle: Q = C + R*(P-C)/|P-C|
      // Q keeps the sign below, so the wrong half skips the square root entirely
      const double flexion_from_center   = flexion_deg   - piece.arc_center_flexion_deg;
      const double abduction_from_center = abduction_deg - piece.arc_center_abduction_deg;
      bool radial_foot_on_piece = abduction_from_center * piece.arc_signed_radius_deg > 0.0;
      if (radial_foot_on_piece) {
        const double scale_to_circle = std::fabs(piece.arc_signed_radius_deg)
            / std::sqrt(flexion_from_center * flexion_from_center
                      + abduction_from_center * abduction_from_center);
        candidate_flexion_deg   = piece.arc_center_flexion_deg   + flexion_from_center   * scale_to_circle;
        candidate_abduction_deg = piece.arc_center_abduction_deg + abduction_from_center * scale_to_circle;
        radial_foot_on_piece = candidate_flexion_deg >= piece.flexion_lo_deg - kToleranceDeg &&
                               candidate_flexion_deg <= piece.flexion_hi_deg + kToleranceDeg;
      }
      if (!radial_foot_on_piece) {
        // Off the arc, and distance along a circle grows with the angle, so the nearer knot wins
        const double to_lo_flexion   = flexion_deg   - piece.flexion_lo_deg;
        const double to_lo_abduction = abduction_deg - piece.abduction_lo_deg;
        const double to_hi_flexion   = flexion_deg   - piece.flexion_hi_deg;
        const double to_hi_abduction = abduction_deg - piece.abduction_hi_deg;
        if (to_hi_flexion * to_hi_flexion + to_hi_abduction * to_hi_abduction <
            to_lo_flexion * to_lo_flexion + to_lo_abduction * to_lo_abduction) {
          candidate_flexion_deg   = piece.flexion_hi_deg;
          candidate_abduction_deg = piece.abduction_hi_deg;
        } else {
          candidate_flexion_deg   = piece.flexion_lo_deg;
          candidate_abduction_deg = piece.abduction_lo_deg;
        }
      }
    }

    const double flexion_gap   = flexion_deg   - candidate_flexion_deg;
    const double abduction_gap = abduction_deg - candidate_abduction_deg;
    const double distance = flexion_gap * flexion_gap + abduction_gap * abduction_gap;
    if (distance < nearest_distance) {
      nearest_distance      = distance;
      nearest_flexion_deg   = candidate_flexion_deg;
      nearest_abduction_deg = candidate_abduction_deg;
    }
  }

  // A rounding result near -1e-11 would flip the direction when the sign is put back
  if (nearest_abduction_deg < 0.0) nearest_abduction_deg = 0.0;
  target_rad[flexion_slot]   = nearest_flexion_deg * kDeg2Rad;
  target_rad[abduction_slot] = abduction_sign * nearest_abduction_deg * kDeg2Rad;
}

// The long j3 floor in rad at flexion x and abduction y in rad, the docs formula term by term:
// beta and lambda as written there, phi the floor before max(0, phi)
// Below the floor the j3 actuator would have to pass its home stop, so a smaller j3 never reaches
// the hand. Abduction enters only through cos y, so the sign of y does not matter
double long_distal_floor_rad(double x, double y)
{
  const std::array<double, 11>& a = kDistalFloorA;
  const std::array<double, 9>& b = kDistalFloorB;
  const double beta = a[0] - x;
  const double sin_beta = std::sin(beta), cos_beta = std::cos(beta), cos_y = std::cos(y);
  const double lambda = a[1] + x + std::atan2(sin_beta + a[2] * cos_y, cos_beta + a[3])
      - std::acos(std::clamp((a[4] + a[5] * sin_beta * cos_y + a[6] * cos_beta)
                                 / std::sqrt(a[7] + a[8] * cos_y * cos_y + a[9] * sin_beta * cos_y + a[10] * cos_beta),
                             -1.0, 1.0));
  const double sin_lambda = std::sin(lambda), cos_lambda = std::cos(lambda);
  const double phi = b[0] - std::atan2(sin_lambda + b[1], cos_lambda + b[2])
      + std::acos(std::clamp((b[3] + b[4] * cos_lambda + b[5] * sin_lambda)
                                 / std::sqrt(b[6] + b[7] * cos_lambda + b[8] * sin_lambda),
                             -1.0, 1.0));
  return std::max(0.0, phi);
}

// JointPosition and JointImpedance share the target space, so they share this
void clamp_active_joint_targets(std::array<double, kActiveJointCount>& target)
{
  // Coupled pairs, as (abduction slot, flexion slot)
  clamp_finger_workspace(target,  2,  1, kThumbBoundary, kThumbFlexionMaxDeg);   // thumb
  clamp_finger_workspace(target,  4,  5, kLongBoundary,  kLongFlexionMaxDeg);    // index
  clamp_finger_workspace(target,  7,  8, kLongBoundary,  kLongFlexionMaxDeg);    // middle
  clamp_finger_workspace(target, 10, 11, kLongBoundary,  kLongFlexionMaxDeg);    // ring
  clamp_finger_workspace(target, 13, 14, kLongBoundary,  kLongFlexionMaxDeg);    // baby

  // Joints outside the coupled pairs, clamped to a constant range
  target[0]  = std::clamp(target[0],  0.0, kThumbJ0MaxDeg * kDeg2Rad);      // thumb j0 CMC
  target[3]  = std::clamp(target[3],  0.0, kThumbJ3MaxDeg * kDeg2Rad);      // thumb j3
  target[6]  = std::clamp(target[6],  0.0, kLongDistalMaxDeg * kDeg2Rad);   // index j3 distal
  target[9]  = std::clamp(target[9],  0.0, kLongDistalMaxDeg * kDeg2Rad);   // middle j3
  target[12] = std::clamp(target[12], 0.0, kLongDistalMaxDeg * kDeg2Rad);   // ring j3
  target[15] = std::clamp(target[15], 0.0, kLongDistalMaxDeg * kDeg2Rad);   // baby j3

  // The long j3 floor follows the pairs clamped above, so it comes last
  for (const LongDistal& distal : kLongDistals) {
    const double floor_rad = long_distal_floor_rad(target[distal.flexion_slot], target[distal.abduction_slot]);
    if (target[distal.distal_slot] < floor_rad) target[distal.distal_slot] = floor_rad;
  }
}

}  // namespace

void JointPositionCommand::clamp()  { clamp_active_joint_targets(target); }
void JointImpedanceCommand::clamp() { clamp_active_joint_targets(target); }

}  // namespace aidin_hand2
