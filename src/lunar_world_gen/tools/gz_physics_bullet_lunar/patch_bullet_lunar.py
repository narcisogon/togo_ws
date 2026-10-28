#!/usr/bin/env python3
"""Apply the two lunar-world patches to a gz-physics7 7.6.0 source tree (classic Bullet).

Patch 1 - velcmd (JointFeatures.cc, SetJointVelocityCommand), from the rover baseline
(tools/gz_physics_bullet_velcmd_fix/, docs/research/rover_baseline.md section 7):
    stock Bullet overwrites the child link's WHOLE angular velocity with axis * value every
    step, destroying the chassis yaw rate the wheels share through their hinges (an artificial
    yaw damper c = 4 I_wheel / dt = 79 N m s/rad at 1 ms; the rover turns at 13 % of the
    command at lunar gravity). Patched: only the joint-axis component of the child's angular
    velocity RELATIVE to the parent is set to the command.

Patch 2 - mesh friction (Base.hh, SDFFeatures.cc, ShapeFeatures.cc):
    stock classic Bullet never applies <surface><friction><ode><mu>/<mu2> to MESH collisions:
    gz-sim builds mesh shapes through ShapeFeatures::AttachMeshShape, which has no access to
    the SDF, so a mesh body keeps btRigidBody's default friction 0.5. gz-sim does, however,
    hand the full model DOM to SDFFeatures::ConstructSdfModel first, whose
    ConstructSdfCollision reads mu/mu2 for every collision but only builds primitives.
    Patched: ConstructSdfCollision records (mu, mu2) of mesh collisions keyed by
    (link id, collision name); AttachMeshShape looks them up and applies them to the link's
    rigid body, as ConstructSdfCollision does for primitives (Bullet friction is per body;
    the last collision of a link wins, exactly as for primitives). Isotropic values (mu ==
    mu2) are set as the body's scalar friction (btCollisionObject::setFriction(mu), anisotropy
    off), so the effective coefficient against another body is the product mu * mu_other on
    a sloped, world-frame terrain mesh too. (The primitives' scheme, setFriction(1) +
    anisotropic (mu, mu2, 1), scales a friction direction's body-z component by 1, which on a
    world-frame mesh incline raises the effective coefficient, e.g. 0.2 -> 0.39 at 20 deg.)
    For mu != mu2 that anisotropic scheme is mirrored.

usage: patch_bullet_lunar.py <gz-physics source dir>     (idempotent; keeps *.orig copies)
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

MARK = "[lunar patch]"

PATCHES = [
    # ------------------------------------------------------------------ 1: velcmd yaw damper
    ("bullet/src/JointFeatures.cc", "velcmd",
     """    const auto &link = this->links.at(jointInfo->childLinkId)->link;
    btTransform trans;
    link->getMotionState()->getWorldTransform(trans);
    btVector3 motion = quatRotate(trans.getRotation(),
      convertVec(gz::math::eigen3::convert(jointInfo->axis)));
    btVector3 angular_vel = motion * static_cast<btScalar>(_value);
    link->setAngularVelocity(angular_vel);""",
     """    const auto &link = this->links.at(jointInfo->childLinkId)->link;
    btTransform trans;
    link->getMotionState()->getWorldTransform(trans);
    btVector3 motion = quatRotate(trans.getRotation(),
      convertVec(gz::math::eigen3::convert(jointInfo->axis)));
    // [lunar patch] velcmd: keep the parent's angular velocity; only the
    // relative angular velocity about the joint axis is set to _value.
    btVector3 parentAngVel(0, 0, 0);
    auto parentIt = this->links.find(jointInfo->parentLinkId);
    if (parentIt != this->links.end() && parentIt->second &&
        parentIt->second->link)
    {
      parentAngVel = parentIt->second->link->getAngularVelocity();
    }
    btVector3 current = link->getAngularVelocity();
    btVector3 relative = current - parentAngVel;
    btVector3 angular_vel = current - motion * motion.dot(relative) +
      motion * static_cast<btScalar>(_value);
    link->setAngularVelocity(angular_vel);"""),
    # ------------------------------------------------------------------ 2a: storage in Base
    ("bullet/src/Base.hh", "mesh-friction",
     """  public: std::unordered_map<std::size_t, JointInfoPtr> joints;
};""",
     """  public: std::unordered_map<std::size_t, JointInfoPtr> joints;

  /// \\brief [lunar patch] mesh friction: SDF <surface><friction><ode> (mu, mu2)
  /// of MESH collisions, keyed by (link id, collision name). Recorded by
  /// ConstructSdfCollision (which cannot build mesh shapes: gz-sim attaches them
  /// later through AttachMeshShape, without the SDF) and applied by
  /// AttachMeshShape to the link's rigid body.
  public: std::map<std::pair<std::size_t, std::string>,
                   std::pair<btScalar, btScalar>> meshFriction;
};"""),
    # ------------------------------------------------------------------ 2b: record (SDF)
    ("bullet/src/SDFFeatures.cc", "mesh-friction",
     """  const auto mu = odeFriction->Get<btScalar>("mu");
  const auto mu2 = odeFriction->Get<btScalar>("mu2");
""",
     """  const auto mu = odeFriction->Get<btScalar>("mu");
  const auto mu2 = odeFriction->Get<btScalar>("mu2");

  // [lunar patch] mesh friction: mesh shapes are attached later by gz-sim through
  // AttachMeshShape (no SDF there); remember this collision's friction for it.
  if (shape == nullptr && geom->MeshShape() != nullptr)
  {
    this->meshFriction[{static_cast<std::size_t>(_linkID), _collision.Name()}] =
      {mu, mu2};
  }
"""),
    # ------------------------------------------------------------------ 2c: apply (shape)
    ("bullet/src/ShapeFeatures.cc", "mesh-friction",
     """  const auto &linkInfo = this->links.at(_linkID);
  const auto &modelID = linkInfo->model;
  const auto &body = linkInfo->link.get();
""",
     """  const auto &linkInfo = this->links.at(_linkID);
  const auto &modelID = linkInfo->model;
  const auto &body = linkInfo->link.get();

  // [lunar patch] mesh friction: honour the SDF <ode><mu>/<mu2> recorded by
  // ConstructSdfCollision. Isotropic friction is the body's scalar friction
  // (effective coefficient = mu * mu_other on any slope); anisotropic friction
  // mirrors the primitives (setFriction(1) + anisotropic (mu, mu2, 1)).
  {
    const auto fit = this->meshFriction.find(
      {static_cast<std::size_t>(_linkID), _name});
    if (fit != this->meshFriction.end())
    {
      const btScalar mu = fit->second.first;
      const btScalar mu2 = fit->second.second;
      if (std::fabs(mu - mu2) <= btScalar(1e-6))
      {
        body->setFriction(mu);
        body->setAnisotropicFriction(btVector3(1, 1, 1));
      }
      else
      {
        body->setFriction(1);
        body->setAnisotropicFriction(btVector3(mu, mu2, 1),
          btCollisionObject::CF_ANISOTROPIC_FRICTION);
      }
      gzmsg << "[lunar patch] mesh collision [" << _name << "]: friction mu "
            << mu << ", mu2 " << mu2 << std::endl;
    }
  }
"""),
    ("bullet/src/ShapeFeatures.cc", "mesh-friction",
     """#include <memory>
""",
     """#include <cmath>
#include <memory>
"""),
]


def apply(src_dir: Path) -> list[str]:
    """Apply all patches; returns a log. Raises SystemExit if a pattern is not found once."""
    log = []
    by_file: dict[str, list[tuple[str, str, str]]] = {}
    for rel, tag, old, new in PATCHES:
        by_file.setdefault(rel, []).append((tag, old, new))
    for rel, items in by_file.items():
        path = src_dir / rel
        txt = path.read_text()
        orig = txt
        for tag, old, new in items:
            if new in txt:
                log.append(f"{rel}: {tag} already applied")
                continue
            n = txt.count(old)
            if n != 1:
                raise SystemExit(f"{rel}: pattern for '{tag}' found {n} times (expected 1); "
                                 "is this gz-physics 7.6.0?")
            txt = txt.replace(old, new)
            log.append(f"{rel}: {tag} applied")
        if txt != orig:
            backup = path.with_name(path.name + ".orig")
            if not backup.exists():
                shutil.copy2(path, backup)
            path.write_text(txt)
    return log


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__)
        return 2
    src = Path(argv[1])
    if not (src / "bullet" / "src" / "JointFeatures.cc").is_file():
        print(f"{src} is not a gz-physics source tree", file=sys.stderr)
        return 2
    for line in apply(src):
        print(line)
    marks = sum((src / rel).read_text().count(MARK) for rel in {p[0] for p in PATCHES})
    print(f"{marks} '{MARK}' markers in the patched files")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
