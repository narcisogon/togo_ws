#!/usr/bin/env python3
"""Patch gz-physics7 (7.6.0) classic-Bullet JointFeatures::SetJointVelocityCommand.

Stock behaviour (bullet/src/JointFeatures.cc): the child link's WHOLE angular velocity is
overwritten with  axis_world * value  every physics step.  For a wheel this throws away the
chassis yaw/roll rate that the wheel shares through the hinge, i.e. every step the wheels'
yaw angular momentum is destroyed.  On a skid-steer rover this is an artificial yaw damper of
  c = n_wheels * I_wheel_perp / dt   (TOGO A300: 4 * 0.01976 / 0.001 = 79 N m s/rad at 1 ms)
that does not depend on gravity, while the traction yaw moment scales with mu*m*g -> at lunar
gravity the rover can hardly turn (13 % of the commanded yaw rate on a mu 0.2 plane).

Patched behaviour: only the joint-axis component of the child's angular velocity RELATIVE to
the parent is set to `value`; the other components are left to the hinge constraint.

usage: patch_bullet_velcmd.py <gz-physics source dir>     (keeps JointFeatures.cc.orig)
"""
import shutil
import sys

src = sys.argv[1].rstrip('/') + '/bullet/src/JointFeatures.cc'
txt = open(src).read()
old = """    const auto &link = this->links.at(jointInfo->childLinkId)->link;
    btTransform trans;
    link->getMotionState()->getWorldTransform(trans);
    btVector3 motion = quatRotate(trans.getRotation(),
      convertVec(gz::math::eigen3::convert(jointInfo->axis)));
    btVector3 angular_vel = motion * static_cast<btScalar>(_value);
    link->setAngularVelocity(angular_vel);"""
new = """    const auto &link = this->links.at(jointInfo->childLinkId)->link;
    btTransform trans;
    link->getMotionState()->getWorldTransform(trans);
    btVector3 motion = quatRotate(trans.getRotation(),
      convertVec(gz::math::eigen3::convert(jointInfo->axis)));
    // [rover-baseline patch] keep the parent's angular velocity: only the
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
    link->setAngularVelocity(angular_vel);"""
if new in txt:
    print('already patched:', src)
    sys.exit(0)
if txt.count(old) != 1:
    sys.exit(f'pattern not found exactly once in {src} (different gz-physics version?)')
shutil.copy(src, src + '.orig')
open(src, 'w').write(txt.replace(old, new))
print('patched', src)
