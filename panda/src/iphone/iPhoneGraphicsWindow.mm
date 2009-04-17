// Filename: iPhoneGraphicsWindow.mm
// Created by:  drose (08Apr09)
//
////////////////////////////////////////////////////////////////////
//
// PANDA 3D SOFTWARE
// Copyright (c) Carnegie Mellon University. All rights reserved.
//
// All use of this software is subject to the terms of the revised BSD
// license. You should have received a copy of this license along
// with this source code in a file named "LICENSE."
//
////////////////////////////////////////////////////////////////////

// We include these system header files first, because there is a
// namescope conflict between them and some other header file that
// gets included later (in particular, TCP_NODELAY must not be a
// #define symbol for these headers to be included properly).

//#include <ApplicationServices/ApplicationServices.h>
#include <UIKit/UIKit.h>

#include "iPhoneGraphicsWindow.h"
#include "config_iphone.h"
#include "iPhoneGraphicsPipe.h"
#include "pStatTimer.h"
#include "glesgsg.h"
#include "keyboardButton.h"
#include "mouseButton.h"
#include "iPhoneGraphicsStateGuardian.h"
#include "iPhoneGraphicsPipe.h"
#include "throw_event.h"
#include "pnmImage.h"
#include "virtualFileSystem.h"
#include "config_util.h"
#include "pset.h"
#include "pmutex.h"

TypeHandle IPhoneGraphicsWindow::_type_handle;

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::Constructor
//       Access: Public
//  Description:
////////////////////////////////////////////////////////////////////
IPhoneGraphicsWindow::
IPhoneGraphicsWindow(GraphicsEngine *engine, GraphicsPipe *pipe, 
                     const string &name,
                     const FrameBufferProperties &fb_prop,
                     const WindowProperties &win_prop,
                     int flags,
                     GraphicsStateGuardian *gsg,
                     GraphicsOutput *host) :
  GraphicsWindow(engine, pipe, name, fb_prop, win_prop, flags, gsg, host)
{
  _gl_view = nil;

  GraphicsWindowInputDevice device =
    GraphicsWindowInputDevice::pointer_and_keyboard(this, "keyboard/mouse");
  _input_devices.push_back(device);
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::Destructor
//       Access: Public, Virtual
//  Description:
////////////////////////////////////////////////////////////////////
IPhoneGraphicsWindow::
~IPhoneGraphicsWindow() {
  if (_gl_view != nil) {
    [ _gl_view release ];
  }
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::set_pointer_in_window
//       Access: Private
//  Description: Indicates the mouse pointer is seen within the
//               window.
////////////////////////////////////////////////////////////////////
void IPhoneGraphicsWindow::
set_pointer_in_window(int x, int y) {
  _input_devices[0].set_pointer_in_window(x, y);
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::set_pointer_out_of_window
//       Access: Private
//  Description: Indicates the mouse pointer is no longer within the
//               window.
////////////////////////////////////////////////////////////////////
void IPhoneGraphicsWindow::
set_pointer_out_of_window() {
  _input_devices[0].set_pointer_out_of_window();
}


////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::begin_frame
//       Access: Public, Virtual
//  Description: This function will be called within the draw thread
//               before beginning rendering for a given frame. It
//               should do whatever setup is required, and return true
//               if the frame should be rendered, or false if it
//               should be skipped.
////////////////////////////////////////////////////////////////////
bool IPhoneGraphicsWindow::
begin_frame(FrameMode mode, Thread *current_thread) {
  PStatTimer timer(_make_current_pcollector);
 
  begin_frame_spam(mode);
  if (_gsg == (GraphicsStateGuardian *)NULL) {
    // not powered up .. just abort..
    iphone_cat.info() << "no gsg\n";
    return false;
  }

  _gsg->reset_if_new();
  _gsg->set_current_properties(&get_fb_properties());

  return _gsg->begin_frame(current_thread);
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::end_frame
//       Access: Public, Virtual
//  Description: This function will be called within the draw thread
//               after rendering is completed for a given frame. It
//               should do whatever finalization is required.
////////////////////////////////////////////////////////////////////
void IPhoneGraphicsWindow::
end_frame(FrameMode mode, Thread *current_thread) {
  end_frame_spam(mode);
 
  if (mode == FM_render) {
    nassertv(_gsg != (GraphicsStateGuardian *)NULL);

    copy_to_textures();
    
    _gsg->end_frame(current_thread);

    if (_gl_view != nil) {
      [_gl_view presentView];
    }
  }
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::begin_flip
//       Access: Public, Virtual
//  Description: This function will be called within the draw thread
//               after end_frame() has been called on all windows, to
//               initiate the exchange of the front and back buffers.
//
//               This should instruct the window to prepare for the
//               flip at the next video sync, but it should not wait.
//
//               We have the two separate functions, begin_flip() and
//               end_flip(), to make it easier to flip all of the
//               windows at the same time.
////////////////////////////////////////////////////////////////////
void IPhoneGraphicsWindow::
end_flip() {
}

void IPhoneGraphicsWindow::
begin_flip() {
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::close_window
//       Access: Protected, Virtual
//  Description: Closes the window right now. Called from the window
//               thread.
////////////////////////////////////////////////////////////////////
void IPhoneGraphicsWindow::
close_window() {
  //  system_close_window();

  WindowProperties properties;
  properties.set_open(false);
  system_changed_properties(properties);

//  release_system_resources(false);
  _gsg.clear();
  _active = false;
  GraphicsWindow::close_window();
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::open_window
//       Access: Protected, Virtual
//  Description: Opens the window right now. Called from the window
//               thread. Returns true if the window is successfully
//               opened, or false if there was a problem.
////////////////////////////////////////////////////////////////////
bool IPhoneGraphicsWindow::
open_window() {
  iphone_cat.info() << "open_window\n";

  nassertr(_gsg == (GraphicsStateGuardian *)NULL, false);

  _gl_view = [ [ EAGLView alloc ] initWithFrame: 
        [ [ UIScreen mainScreen ] applicationFrame ] 
    ]; 
  IPhoneGraphicsPipe *iphonepipe = DCAST(IPhoneGraphicsPipe, _pipe);
  nassertr(iphonepipe != NULL, false);

  iphonepipe->_view_controller.view = _gl_view;
  [ _gl_view layoutSubviews ];

  WindowProperties req_properties = _properties;

  _gsg = new IPhoneGraphicsStateGuardian(_engine, _pipe, NULL);

  _properties.set_foreground(true);
  _properties.set_minimized(false);
  _properties.set_open(true);
  _is_valid = true;

  return true;
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::process_events
//       Access: Protected, Virtual
//  Description: Required event upcall, used to dispatch window and
//               application events back into panda.
////////////////////////////////////////////////////////////////////
void IPhoneGraphicsWindow::
process_events() {
  GraphicsWindow::process_events();
}

////////////////////////////////////////////////////////////////////
//     Function: IPhoneGraphicsWindow::set_properties_now
//       Access: Public, Virtual
//  Description: Applies the requested set of properties to the
//               window, if possible, for instance to request a change
//               in size or minimization status.
//
//               The window properties are applied immediately, rather
//               than waiting until the next frame. This implies that
//               this method may *only* be called from within the
//               window thread.
//
//               The properties that have been applied are cleared
//               from the structure by this function; so on return,
//               whatever remains in the properties structure are
//               those that were unchanged for some reason (probably
//               because the underlying interface does not support
//               changing that property on an open window).
////////////////////////////////////////////////////////////////////
void IPhoneGraphicsWindow::
set_properties_now(WindowProperties &properties) {
  if (iphone_cat.is_debug()) {
    iphone_cat.debug()
      << "------------------------------------------------------\n";
    iphone_cat.debug()
      << "set_properties_now " << properties << "\n";
  }
 
  GraphicsWindow::set_properties_now(properties);
}
