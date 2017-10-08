/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "SocketPort.hpp"
#include "Net/SocketError.hpp"
#include "IO/DataHandler.hpp"

#ifdef HAVE_POSIX
#include "IO/Async/GlobalIOThread.hpp"
#endif

#include <assert.h>

SocketPort::~SocketPort()
{
  Close();
}

void
SocketPort::Close()
{
  BufferedPort::BeginClose();

#ifdef HAVE_POSIX
  if (socket.IsDefined()) {
    io_thread->LockRemove(socket.ToFileDescriptor());
    socket.Close();
  }
#else
  assert(!thread.IsInside());

  if (thread.IsDefined()) {
    thread.BeginStop();
    thread.Join();
  }

  if (socket.IsDefined())
    socket.Close();
#endif

  BufferedPort::EndClose();

  StateChanged();
}

void
SocketPort::Set(SocketDescriptor &&_socket)
{
  assert(!socket.IsDefined());
  assert(_socket.IsDefined());
#ifndef HAVE_POSIX
  assert(!thread.IsDefined());
#endif

  socket = std::move(_socket);

  /* register the socket in then IOThread or the SocketThread */
#ifdef HAVE_POSIX
  io_thread->LockAdd(socket.ToFileDescriptor(), Poll::READ, *this);
#else
  thread.Start(socket);
#endif

  StateChanged();
}

bool
SocketPort::OpenUDPListener(unsigned port)
{
  SocketDescriptor s;
  if (!s.CreateUDPListener(port))
    return false;

  Set(std::move(s));
  return true;
}

PortState
SocketPort::GetState() const
{
  return socket.IsDefined()
    ? PortState::READY
    : PortState::FAILED;
}

bool
SocketPort::Drain()
{
  /* writes are synchronous */
  return true;
}

size_t
SocketPort::Write(const void *data, size_t length)
{
  if (!socket.IsDefined())
    return 0;

  ssize_t nbytes = socket.Write((const char *)data, length);

  if (nbytes < 0 && IsSocketBlockingError()) {
    /* writing to the socket blocks; wait and retry */

    if (socket.WaitWritable(1000) <= 0)
      return 0;

    nbytes = socket.Write(data, length);
  }

  return nbytes < 0 ? 0 : nbytes;
}

unsigned
SocketPort::GetBaudrate() const
{
  return 0;
}

bool
SocketPort::SetBaudrate(unsigned baud_rate)
{
  return true;
}

bool
SocketPort::OnSocketEvent(SocketDescriptor _socket, unsigned mask)
{
  assert(_socket == socket);

  char buffer[1024];
  ssize_t nbytes = socket.Read(buffer, sizeof(buffer));
  if (nbytes <= 0) {
    socket.Close();
    StateChanged();
    return false;
  }

  DataReceived(buffer, nbytes);
  return true;
}
