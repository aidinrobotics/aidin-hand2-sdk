// JointPositionCommand::clamp, pulling a target into each finger's reachable workspace
//
// The parallel wrist couples abduction and flexion, so the measured boundary is a chain of
// line and arc pieces. Inside is left alone, outside is projected to the nearest point
// Past the last knot the boundary is a zero-abduction floor up to the flexion limit
// Abduction is symmetric, handled as an absolute value with the sign put back
// The uncoupled joints, long j3 and thumb j0 and j3, get a constant limit
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
// Rounding the arc coefficients to four digits misses the tabled knot by 1e-4 deg, which
// reads a target on the boundary as outside
enum class PieceKind { Line, Arc };
// Both boundaries have four pieces, so extending a table means changing this too
constexpr std::size_t kPieceCount = 4;

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

// Shared by index, middle, ring and baby, tuned on the 2026-07-30 left hand
constexpr std::array<BoundaryPiece, kPieceCount> kLongBoundary{{
    // kind            flexion lo    hi   abduction lo   hi    arc centre flexion  centre abduction  signed radius
    {PieceKind::Arc,    0.00,  10.60,    0.40,  27.50, -117.5448848530,   62.0000287617, -132.7078125000},
    {PieceKind::Line,  10.60,  54.60,   27.50,  27.50,    0.0,             0.0,             0.0         },
    {PieceKind::Arc,   54.60,  92.68,   27.50,  13.25,  -71.1198326410, -366.4639071557,  413.5372250000},
    {PieceKind::Arc,   92.68,  95.46,   13.25,   0.00,  143.6789848190,   17.0335266262,  -51.1391388889},
}};
// Same tuning round
// The plateau reaches 45.1 deg, past the nominal URDF limit, because the measured range is
// wider than the nominal one and the measurement is what the boundary follows
constexpr std::array<BoundaryPiece, kPieceCount> kThumbBoundary{{
    {PieceKind::Arc,    0.00,   9.00,    0.60,  29.60, -425.8806321979,  148.6664030959, -450.8857644759},
    {PieceKind::Arc,    9.00,  50.00,   29.60,  45.10,  -55.2563274548,  261.5441564934, -240.6802180268},
    {PieceKind::Arc,   50.00,  65.00,   45.10,  37.30,   30.1694515845,  -11.3587469529,   59.8401266539},
    {PieceKind::Arc,   65.00,  76.23,   37.30,   0.00,  251.7767940919,   73.1928136100, -190.1942819332},
}};

// The flexion limit is the last knot itself, so nothing is allowed past what was measured
// That leaves the zero-abduction floor with no length, and the knot as the projection candidate
constexpr double kLongFlexionMaxDeg  = 95.46;
constexpr double kThumbFlexionMaxDeg = 76.23;

// Constant limits in degrees for the uncoupled joints
constexpr double kLongDistalMaxDeg = 90.0;   // long finger j3, URDF 89
constexpr double kThumbJ3MaxDeg    = 70.0;   // thumb j3, URDF 70.53
constexpr double kThumbJ0MaxDeg    = 110.0;  // thumb j0 CMC, URDF 111.5

// Clamps one finger's coupled pair, the table starting at flexion 0 with no gap between pieces
void clamp_finger_workspace(std::array<double, kActiveJointCount>& target_rad,
                            int abduction_slot, int flexion_slot,
                            const std::array<BoundaryPiece, kPieceCount>& boundary,
                            double flexion_max_deg)
{
  const double flexion_deg = std::max(0.0, target_rad[flexion_slot] / kDeg2Rad);
  const double commanded_abduction_deg = target_rad[abduction_slot] / kDeg2Rad;
  // Symmetric, so the sign is put back at the end
  const double abduction_deg  = std::fabs(commanded_abduction_deg);
  const double abduction_sign = commanded_abduction_deg < 0.0 ? -1.0 : 1.0;

  // Past the last knot no piece matches, and the projection below takes over
  for (std::size_t index = 0; index < kPieceCount; ++index) {
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
  double nearest_flexion_deg = std::clamp(flexion_deg, boundary[kPieceCount - 1].flexion_hi_deg,
                                          flexion_max_deg);
  double nearest_abduction_deg = 0.0;
  double nearest_distance = (flexion_deg - nearest_flexion_deg) * (flexion_deg - nearest_flexion_deg)
                          + abduction_deg * abduction_deg;

  for (std::size_t index = 0; index < kPieceCount; ++index) {
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

// JointPosition and JointImpedance share the target space, so they share this
void clamp_active_joint_targets(std::array<double, kActiveJointCount>& target)
{
  // Coupled pairs, as (abduction slot, flexion slot)
  clamp_finger_workspace(target,  2,  1, kThumbBoundary, kThumbFlexionMaxDeg);   // thumb
  clamp_finger_workspace(target,  4,  5, kLongBoundary,  kLongFlexionMaxDeg);    // index
  clamp_finger_workspace(target,  7,  8, kLongBoundary,  kLongFlexionMaxDeg);    // middle
  clamp_finger_workspace(target, 10, 11, kLongBoundary,  kLongFlexionMaxDeg);    // ring
  clamp_finger_workspace(target, 13, 14, kLongBoundary,  kLongFlexionMaxDeg);    // baby

  // Uncoupled joints, clamped to a constant range
  target[0]  = std::clamp(target[0],  0.0, kThumbJ0MaxDeg * kDeg2Rad);      // thumb j0 CMC
  target[3]  = std::clamp(target[3],  0.0, kThumbJ3MaxDeg * kDeg2Rad);      // thumb j3
  target[6]  = std::clamp(target[6],  0.0, kLongDistalMaxDeg * kDeg2Rad);   // index j3 distal
  target[9]  = std::clamp(target[9],  0.0, kLongDistalMaxDeg * kDeg2Rad);   // middle j3
  target[12] = std::clamp(target[12], 0.0, kLongDistalMaxDeg * kDeg2Rad);   // ring j3
  target[15] = std::clamp(target[15], 0.0, kLongDistalMaxDeg * kDeg2Rad);   // baby j3
}

}  // namespace

void JointPositionCommand::clamp()  { clamp_active_joint_targets(target); }
void JointImpedanceCommand::clamp() { clamp_active_joint_targets(target); }

}  // namespace aidin_hand2
