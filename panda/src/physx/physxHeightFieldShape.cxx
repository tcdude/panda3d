/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file physxHeightFieldShape.cxx
 * @author enn0x
 * @date 2009-10-15
 */

#include "physxHeightFieldShape.h"
#include "physxHeightFieldShapeDesc.h"

TypeHandle PhysxHeightFieldShape::_type_handle;

/**
 *
 */
void PhysxHeightFieldShape::
link(NxShape *shapePtr) {

  _ptr = shapePtr->isHeightField();
  _ptr->userData = this;
  _error_type = ET_ok;

  set_name(shapePtr->getName());

  PhysxActor *actor = (PhysxActor *)_ptr->getActor().userData;
  actor->_shapes.add(this);
}

/**
 *
 */
void PhysxHeightFieldShape::
unlink() {

  _ptr->userData = nullptr;
  _error_type = ET_released;

  PhysxActor *actor = (PhysxActor *)_ptr->getActor().userData;
  actor->_shapes.remove(this);
}

/**
 * Saves the state of the shape object to a descriptor.
 */
void PhysxHeightFieldShape::
save_to_desc(PhysxHeightFieldShapeDesc &shapeDesc) const {

  nassertv(_error_type == ET_ok);
  _ptr->saveToDesc(shapeDesc._desc);
}
