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

#include "ProfileListDialog.hpp"
#include "Dialogs/Message.hpp"
#include "Dialogs/TextEntry.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Widget/ListWidget.hpp"
#include "Form/Button.hpp"
#include "Screen/Canvas.hpp"
#include "Screen/Layout.hpp"
#include "OS/FileUtil.hpp"
#include "LocalPath.hpp"
#include "Profile/Map.hpp"
#include "Profile/File.hpp"
#include "UIGlobals.hpp"
#include "Look/DialogLook.hpp"
#include "Language/Language.hpp"

#include <vector>

#include <assert.h>
#include <windef.h> /* for MAX_PATH */

/* this macro exists in the WIN32 API */
#ifdef DELETE
#undef DELETE
#endif

class ProfileListWidget final
  : public ListWidget, private ActionListener {

  struct ListItem {
    StaticString<32> name;
    StaticString<MAX_PATH> path;

    ListItem(const TCHAR *_name, const TCHAR *_path)
      :name(_name), path(_path) {}

    bool operator<(const ListItem &i2) const {
      return StringCollate(name, i2.name) < 0;
    }
  };

  class ProfileFileVisitor: public File::Visitor
  {
    std::vector<ListItem> &list;

  public:
    ProfileFileVisitor(std::vector<ListItem> &_list):list(_list) {}

    void Visit(const TCHAR* path, const TCHAR* filename) {
      list.emplace_back(filename, path);
    }
  };

  enum Buttons {
    NEW,
    COPY,
    DELETE,
  };

  WndForm *form;
  WndButton *copy_button, *delete_button;

  std::vector<ListItem> list;

public:
  void CreateButtons(WidgetDialog &dialog);

  void SelectPath(const TCHAR *path);

private:
  void UpdateList();

  gcc_pure
  int FindPath(const TCHAR *path) const;

  void NewClicked();
  void CopyClicked();
  void DeleteClicked();

public:
  /* virtual methods from class Widget */
  virtual void Prepare(ContainerWindow &parent,
                       const PixelRect &rc) override;
  virtual void Unprepare() override;

protected:
  /* virtual methods from ListItemRenderer */
  virtual void OnPaintItem(Canvas &canvas, const PixelRect rc,
                           unsigned idx) override;

private:
  /* virtual methods from class ActionListener */
  virtual void OnAction(int id) override;
};

gcc_pure
static UPixelScalar
GetRowHeight(const DialogLook &look)
{
  return 2 * Layout::GetTextPadding() + look.list.font->GetHeight();
}

void
ProfileListWidget::UpdateList()
{
  list.clear();

  ProfileFileVisitor pfv(list);
  VisitDataFiles(_T("*.prf"), pfv);

  unsigned len = list.size();

  if (len > 0)
    std::sort(list.begin(), list.end());

  ListControl &list_control = GetList();
  list_control.SetLength(len);
  list_control.Invalidate();

  const bool empty = list.empty();
  copy_button->SetEnabled(!empty);
  delete_button->SetEnabled(!empty);
}

int
ProfileListWidget::FindPath(const TCHAR *path) const
{
  for (unsigned n = list.size(), i = 0u; i < n; ++i)
    if (StringIsEqual(path, list[i].path))
      return i;

  return -1;
}

void
ProfileListWidget::SelectPath(const TCHAR *path)
{
  auto i = FindPath(path);
  if (i >= 0)
    GetList().SetCursorIndex(i);
}

void
ProfileListWidget::CreateButtons(WidgetDialog &dialog)
{
  form = &dialog;

  dialog.AddButton(_("New"), *this, NEW);
  copy_button = dialog.AddButton(_("Copy"), *this, COPY);
  delete_button = dialog.AddButton(_("Delete"), *this, DELETE);
}

void
ProfileListWidget::Prepare(ContainerWindow &parent, const PixelRect &rc)
{
  const DialogLook &look = UIGlobals::GetDialogLook();
  CreateList(parent, look, rc, GetRowHeight(look));
  UpdateList();
}

void
ProfileListWidget::Unprepare()
{
  DeleteWindow();
}

void
ProfileListWidget::OnPaintItem(Canvas &canvas, const PixelRect rc, unsigned i)
{
  assert(i < list.size());

  const DialogLook &look = UIGlobals::GetDialogLook();
  const Font &name_font = *look.list.font;

  canvas.Select(name_font);

  canvas.DrawClippedText(rc.left + Layout::GetTextPadding(),
                         rc.top + Layout::GetTextPadding(), rc, list[i].name);
}

inline void
ProfileListWidget::NewClicked()
{
  StaticString<64> name;
  name.clear();
  if (!TextEntryDialog(name, _("Profile name")))
      return;

  StaticString<80> filename;
  filename = name;
  filename += _T(".prf");

  StaticString<MAX_PATH> path;
  LocalPath(path.buffer(), filename);

  if (!File::CreateExclusive(path)) {
    ShowMessageBox(name, _("File exists already."), MB_OK|MB_ICONEXCLAMATION);
    return;
  }

  UpdateList();
  SelectPath(path);
}

inline void
ProfileListWidget::CopyClicked()
{
  assert(GetList().GetCursorIndex() < list.size());

  const unsigned index = GetList().GetCursorIndex();
  const TCHAR *old_path = list[index].path;
  const TCHAR *old_filename = list[index].name;

  ProfileMap data;
  if (!Profile::LoadFile(data, old_path)) {
    ShowMessageBox(old_filename, _("Failed to load file."),
                   MB_OK|MB_ICONEXCLAMATION);
    return;
  }

  StaticString<64> new_name;
  new_name.clear();
  if (!TextEntryDialog(new_name, _("Profile name")))
      return;

  StaticString<80> new_filename;
  new_filename = new_name;
  new_filename += _T(".prf");

  StaticString<MAX_PATH> new_path;
  LocalPath(new_path.buffer(), new_filename);

  if (File::ExistsAny(new_path)) {
    ShowMessageBox(new_name, _("File exists already."),
                   MB_OK|MB_ICONEXCLAMATION);
    return;
  }

  if (!Profile::SaveFile(data, new_path)) {
    ShowMessageBox(new_name, _("Failed to save file."),
                   MB_OK|MB_ICONEXCLAMATION);
    return;
  }

  UpdateList();
  SelectPath(new_path);
}

inline void
ProfileListWidget::DeleteClicked()
{
  assert(GetList().GetCursorIndex() < list.size());

  StaticString<256> tmp;
  StaticString<256> tmp_name(list[GetList().GetCursorIndex()].name.c_str());
  if (tmp_name.length() > 4)
    tmp_name.Truncate(tmp_name.length() - 4);

  tmp.Format(_("Delete \"%s\"?"),
             tmp_name.c_str());
  if (ShowMessageBox(tmp, _("Delete"), MB_YESNO) != IDYES)
    return;

  File::Delete(list[GetList().GetCursorIndex()].path);
  UpdateList();
}

void
ProfileListWidget::OnAction(int id)
{
  switch ((Buttons)id) {
  case NEW:
    NewClicked();
    break;

  case COPY:
    CopyClicked();
    break;

  case DELETE:
    DeleteClicked();
    break;
  }
}

void
ProfileListDialog()
{
  ProfileListWidget widget;
  WidgetDialog dialog(UIGlobals::GetDialogLook());
  dialog.CreateFull(UIGlobals::GetMainWindow(), _("Profiles"), &widget);
  widget.CreateButtons(dialog);
  dialog.AddButton(_("Close"), mrOK);

  dialog.ShowModal();
  dialog.StealWidget();
}