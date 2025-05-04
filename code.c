#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <pwd.h>
#include <Xm/Xm.h>           /* Motif Toolkit */
#include <Mrm/MrmPublic.h>   /* Mrm */
#include <Xm/Container.h>
#include <Xm/SelectioB.h>
#include <Xm/TextF.h>
#include "filemanager.h"

void 
gotoCB(Widget widget, XtPointer i1, XtPointer cb)
{
  Widget text;
  char *str;

  text = XtNameToWidget(widget, "*Text");

  str = XmTextFieldGetString(text);

  readdirCB(widget, str, NULL);

  XtFree(str);
}

void 
manageCB(Widget widget, Widget w_to_manage, XtPointer callback_data)
{
  if (w_to_manage != (Widget) NULL)
    XtManageChild(w_to_manage);
}

void 
viewCB(Widget widget, char   *type, XtPointer callback_data)
{
  XtEnum size, spatial;

  if (strcmp(type, "large") == 0) {
    size = XmLARGE_ICON;
    spatial = XmSPATIAL;
  } else if (strcmp(type, "small") == 0) {
    size = XmSMALL_ICON;
    spatial = XmSPATIAL;
  } else if (strcmp(type, "detail") == 0) {
    size = XmSMALL_ICON;
    spatial = XmDETAIL;
  }

  XtVaSetValues(fileviewer,
		XmNlayoutType, spatial,
		XmNentryViewType, size,
		0, 0);

  fixViewerSize(XtParent(fileviewer), NULL, NULL);

  XmContainerRelayout(fileviewer);
}

void 
quitCB(Widget widget, char *tag, XmAnyCallbackStruct *callback_data)
{
  exit(0);
}

char* 
fullpath(char *filename)
{
  char buf[1024];
  char *rstr;

  /* This is for UNIX,  write something else for a non-UNIX OS */
  if (strcmp(currentdir, "/") == 0)
    sprintf(buf, "/%s", filename);
  else if (filename[0] == '/')
    sprintf(buf, "%s%s", currentdir, filename);
  else
    sprintf(buf, "%s/%s", currentdir, filename);

  rstr = XtNewString(buf);

  return(rstr);
}


void 
showHiddenCB(Widget widget, XtPointer ignore,
	     XmToggleButtonCallbackStruct *callback_data)
{
  showHidden = callback_data -> set;

  read_directory((Widget) NULL, ".");
}

void 
newFolder(Widget widget, XtPointer ignore, XtPointer ignore2)
{
  char buf[256];
  int i = 0;
  int status;
  Boolean done = False;
  struct stat statbuf;

  while(! done) {
    sprintf(buf, "%s/unnamed%d", currentdir, i);
    status = stat(buf, &statbuf);
    if (status != 0) {
      if (errno == ENOENT)
	done = True;
      else
	return; /* Bad error */
    }
    if (i >= 100) return;
    i++;
  }

  mkdir(buf, 0755);

}
     
void 
deleteItem(Widget widget, XtPointer ignore, XtPointer ignore2)
{
  WidgetList selected;
  Cardinal count;
  int i;

  /* First get list of selected items. */
  XtVaGetValues(fileviewer, 
		XmNselectedObjects, &selected,
		XmNselectedObjectCount, &count,
		NULL, NULL);

  if (count <= 0) return;
 
  for(i = 0; i < count; i++) {
    char buf[256];
    sprintf(buf, deleteCommand, getPathFromIcon(selected[i]));
    system(buf);
  }

  XtVaSetValues(fileviewer, XmNselectedObjectCount, 0, NULL, NULL);
}


/* $XConsortium: convert.c /main/5 1995/07/15 20:45:16 drk $ */
#include <Xm/Xm.h>
#include <Xm/Container.h>
#include <Xm/Transfer.h>
#include "filemanager.h"

void 
targetConvertCallback(Widget wid, XtPointer ignore,
		      XmConvertCallbackStruct *cs)
{
  Atom XA_TARGETS = XInternAtom(XtDisplay(wid), XmSTARGETS, False);
  Atom XA_FILE = XInternAtom(XtDisplay(wid), XmSFILE, False);
  Atom XA_FILENAME = XInternAtom(XtDisplay(wid), XmSFILE_NAME, False);
  Atom XA_MOTIF_EXPORTS = 
    XInternAtom(XtDisplay(wid), XmS_MOTIF_EXPORT_TARGETS, False);
  Atom XA_MOTIF_REQUIRED = 
    XInternAtom(XtDisplay(wid), XmS_MOTIF_CLIPBOARD_TARGETS, False);
  Atom XA_MOTIF_DROP =
    XInternAtom(XtDisplay(wid), XmS_MOTIF_DROP, False);

  if (cs -> selection == XA_MOTIF_DROP &&
      (cs -> target == XA_TARGETS ||
       cs -> target == XA_MOTIF_EXPORTS ||
       cs -> target == XA_MOTIF_REQUIRED)) {
    Atom *targs;
    targs = (Atom *) XtMalloc(sizeof(Atom) * 2);
    targs[0] = XA_FILE;
    targs[1] = XA_FILENAME;
    cs -> value = (XtPointer) targs;
    cs -> length = 2;
    cs -> type = XA_ATOM;
    cs -> format = 32;
    cs -> status = XmCONVERT_MERGE;
  } else if (cs -> target == XA_FILE ||
	     cs -> target == XA_FILENAME) {
    if (cs -> location_data == NULL) {
      WidgetList selected;
      Cardinal count;
      int i;

      /* First get list of selected items. */
      XtVaGetValues(fileviewer, 
		    XmNselectedObjects, &selected,
		    XmNselectedObjectCount, &count,
		    NULL, NULL);

      if (count > 0) {
	char *rval = NULL;
	int curpos = 0;

	for(i = 0; i < count; i++) {
	  char *path;
	  int pathlen;

	  path = getPathFromIcon(selected[i]);
	  pathlen = strlen(path) + 1;
	  rval = XtRealloc(rval, curpos + pathlen);
	  /* This will include the terminating NULL byte.  Important,
	     do not remove this NULL byte,  it acts as a separator */
	  strncpy(rval, path, pathlen);
	  curpos += pathlen;
	}
	cs -> format = 8;
	cs -> length = curpos - 1;
	cs -> type = XA_STRING;
	cs -> status = XmCONVERT_DONE;
      } else {
	cs -> status = XmCONVERT_REFUSE;
      }
    } else {
      char *path;

      path = getPathFromIcon(cs -> location_data);

      cs -> value = path;
      if (path != NULL)
	cs -> length = strlen(path);
      else
	cs -> length = 0;
      cs -> format = 8;
      cs -> type = XA_STRING;
      cs -> status = XmCONVERT_DONE;
    }
  } 
}

void 
targetDestinationCallback(Widget w, XtPointer ignore,
			  XmDestinationCallbackStruct *cs)
{
  /* Later */
}

#ifdef REV_INFO
#ifndef lint
static char rcsid[] = "$XConsortium: filemanager.c /main/6 1995/07/14 09:41:39 drk $"
#endif
#endif

#include <stdlib.h>
#include <stdio.h>
#include <Xm/Xm.h>           /* Motif Toolkit */
#include <Xm/Container.h>
#include "filemanager.h"

Display  	*display;
XtAppContext  	app_context;
Widget toplevel, mainW, dirOM, fileviewer;
Widget gotoDialog, displayLabel;

static int ErrorHandler(Display *display, XErrorEvent *event);
static void UpdateDir(XtPointer, XtIntervalId*);

#define APP_CLASS "XmdFilemanager"
time_t ltm = 0;
unsigned int updateTime;
char *deleteCommand;
XrmQuark app_class_quark;
XrmQuark app_quark;

char * fallback_resources[] = {
"*updateTime:				2000", /* Check on directory every two seconds */
"*deleteCommand:			rm %s",
"*types.o.largeIcon:			obj.xpm",
"*types.c.largeIcon:			code.xpm",
"*types.h.largeIcon:			code.xpm",
"*types.xpm.largeIcon:			pix.xpm",
"*types.xpm.smallIcon:			s_pix.xpm",
"*types.xpm.action:			vueicon -f %s",
"*types.filemanager.action:		%s",
"*types.default_file.largeIcon:		file.xpm",
"*types.default_file.smallIcon:		s_file.xpm",
"*types.default_dir.largeIcon:		dir.xpm",
"*types.default_dir.smallIcon:		s_dir.xpm",
"*types.default_exec.largeIcon:		exec.xpm",
"*types.default_exec.smallIcon:		s_exec.xpm",
"*types.default_none.largeIcon:		none.xpm",
"*types.default_none.smallIcon:		s_none.xpm",
"*types.default_file.action:		$EDITOR %s",
"*types.default_exec.action:		xterm -e %s",
"*XmContainer.OutlineButton.shadowThickness: 0",
/* "*XmContainer.OutlineButton*armPixmap:	arm.xpm", */
"*XmContainer*foreground:		black",
"*XmContainer*background:		gray90",
"*XmContainer.selectColor:		lightblue",
"*XmContainer.outlineIndentation:	30",
"*XmContainer.outlineColumnWidth:	6cm",
"*XmContainer.detailTabList:		+3cm,+3cm,+3cm",
"*XmIconGadget.renderTable:		*helvetica-bold-r-*-10-*",
"*XmIconGadget.shadowThickness:		0",
"*show_hidden.labelString:		Show Hidden Files",
"*show_hidden.indicatorOn:		XmINDICATOR_CHECK",
"*show_hidden.indicatorSize:		15",
"*help_manager.helpFile:		filemanager",
NULL
};

/*
 *  Main program
 */
int
main(int argc, char* argv[])
{
  Arg		args[10];
  int 		n = 0;
  Pixel		fg, bg;
  Display	*display;
  int		time;
  char		*str;
  int 		i;

  XtSetArg(args[n], XmNallowShellResize, True);  n++;
  toplevel = XtVaAppInitialize(&app_context, APP_CLASS, NULL, 0, &argc, argv,
			       fallback_resources, NULL);

  app_class_quark = XrmStringToQuark(APP_CLASS);
  app_quark = XrmStringToQuark(argv[0]);

  mainW = CreateInterface("main", toplevel);

  /* First try the resource database,  then use the fallbacks below */
  str = XGetDefault(XtDisplay(toplevel), APP_CLASS, "updateTime");
  if (str == NULL)
    str = XGetDefault(XtDisplay(toplevel), argv[0], "updateTime");
  if (str == NULL)
    updateTime = 2000;
  else
    updateTime = atoi(str);

  str = XGetDefault(XtDisplay(toplevel), APP_CLASS, "deleteCommand");
  if (str == NULL)
    str = XGetDefault(XtDisplay(toplevel), argv[0], "deleteCommand");
  if (str == NULL)
    deleteCommand = "rm %s";
  else
    deleteCommand = str;

  /* Find the widgets in the hierarchy */
  dirOM = XtNameToWidget(mainW, "*dirOM");

  dirLabel = (Widget *) XtMalloc(sizeof(Widget) * ndirLabel);
  for (i = 0; i < ndirLabel; i++) {
    char buf[16];
    sprintf(buf, "*l%d", i);
    dirLabel[i] = XtNameToWidget(mainW, buf);
    paths[i] = NULL;
  }

  fileviewer = XtNameToWidget(mainW, "*container");
  gotoDialog = XtNameToWidget(mainW, "*gotoDialog");
  
  displayLabel = XtNameToWidget(mainW, "*Where");

  XtVaGetValues(fileviewer, XmNforeground, &bg,	
		XmNbackground, &fg, NULL, NULL);

  XtManageChild(mainW);

  /* Install our special error handler */
  XSetErrorHandler((XErrorHandler) ErrorHandler);

  /* Add the UTM callbacks on the container area */
  XtAddCallback(fileviewer, XmNdestinationCallback, 
		(XtCallbackProc) targetDestinationCallback, NULL);
  XtAddCallback(fileviewer, XmNconvertCallback, 
		(XtCallbackProc) targetConvertCallback, NULL);
  XtAddCallback(XtParent(fileviewer), XmNresizeCallback,
		fixViewerSize, NULL);

  readdirCB(fileviewer, ".", NULL);

  XtRealizeWidget(toplevel);

  XmContainerRelayout(fileviewer);

  XtAppAddTimeOut(app_context, updateTime,
		  UpdateDir, (XtPointer) 0);

  XtAppMainLoop(app_context);

  return 0;    /* make compiler happy */
}

/* Adjust the size of the underlying container widget to be
   at least large enough to fit in the clipwindow */
void 
fixViewerSize(Widget w, XtPointer i1, XtPointer i2) 
{
  Dimension width, height, container_w, container_h;

  /* Get the correct dimensions of the clipwindow */
  XtVaGetValues(w, XtNwidth, &width, XtNheight, &height, NULL, NULL);

  /* Get the correct dimensions of the container */
  XtVaGetValues(fileviewer, XtNwidth, &container_w,
		XtNheight, &container_h, NULL, NULL);

  XtVaSetValues(fileviewer, XtNwidth, width, NULL, NULL);
  if (container_h < height) 
    XtVaSetValues(fileviewer, XtNheight, height, NULL, NULL);
}

/* Error handler for X protocol errors.  Continue after error */

static int 
ErrorHandler(Display *display, XErrorEvent *event)
{
  char errortext[100];
  XmString tmp;

  XGetErrorText(display, event -> error_code, errortext, 100);

  printf("X Protocol error: %s\n", errortext);
  printf("XID %ld serial %ld major %d minor %d\n",
	 event -> resourceid,
	 event -> serial,
	 event -> request_code,
	 event -> minor_code);

  return 0;
}

/* Check for modification of current directory.  We don't handle updates
   in subdirectories (at least yet),  but this *is* free software */
static void 
UpdateDir(XtPointer data, XtIntervalId *id) 
{
  struct stat buf;

  XtAppAddTimeOut(app_context, updateTime, 
		  (XtTimerCallbackProc) UpdateDir, 0);

  if (stat(currentdir, &buf) != 0) {
    printf("Can't stat directory: %s\n", currentdir);
    return;
  }

  if (ltm != buf.st_ctime) {
    read_directory((Widget) NULL, ".");
  }
}


/* $XConsortium: filemanager.h /main/5 1995/07/15 20:45:20 drk $ */
#ifndef _filemanager_h
#define _filemanager_h

#include <dirent.h>
#include <sys/stat.h>

typedef struct _FileInfo {
  char *name;
  struct stat statbuf;
  Widget icon;      
  Boolean dirRead;
} FileInfoRec;

extern Widget fileviewer, dirOM, toplevel, mainW, gotoDialog;
extern char *currentdir;
extern Widget *dirLabel, displayLabel;
extern int ndirLabel;
extern char *paths[];
extern FileInfoRec *FI;
extern WidgetList IconGadgets;
extern int validFI;
extern Boolean showHidden;
extern time_t ltm;
extern char *deleteCommand;
extern XrmQuark app_class_quark;
extern XrmQuark app_quark;

extern Widget CreateInterface(char*, Widget);
extern char* fullpath(char*);
extern void readdirCB(Widget, char*, XmAnyCallbackStruct *);
extern void outlineCB(Widget, char*, XmContainerOutlineCallbackStruct *);
extern void manageCB(Widget, Widget, XtPointer);
extern void viewCB(Widget, char*, XtPointer);
extern void gotoCB(Widget widget, XtPointer i1, XtPointer cb);
extern void selectCB(Widget w, XtPointer ignore, XtPointer cb);
extern void quitCB(Widget, char*, XmAnyCallbackStruct*);
extern void newFolder(Widget widget, XtPointer ignore, XtPointer ignore2);
extern void deleteItem(Widget widget, XtPointer ignore, XtPointer ignore2);
extern void targetDestinationCallback(Widget, XtPointer,
				      XmDestinationCallbackStruct*);
extern void targetConvertCallback(Widget, XtPointer,
				  XmConvertCallbackStruct*);
extern char *getPathFromIcon(Widget);
extern void fixViewerSize(Widget w, XtPointer i1, XtPointer i2);
extern void showHiddenCB(Widget widget, XtPointer ignore,
			 XmToggleButtonCallbackStruct *callback_data);
extern void read_directory(Widget, char*);
extern int process_single_entry(Widget, char*, int, struct dirent*);

#endif /* _filemanager_h */



#ifdef REV_INFO
#ifndef lint
static char rcsid[] = "$XConsortium: interface.c /main/6 1995/07/14 09:41:50 drk $"
#endif
#endif

#include <stdio.h>
#include <Xm/Xm.h>           /* Motif Toolkit */
#include <Xm/CascadeBG.h>
#include <Xm/Container.h>
#include <Xm/Form.h>
#include <Xm/LabelG.h>
#include <Xm/MainW.h>
#include <Xm/MessageB.h>
#include <Xm/PushBG.h>
#include <Xm/RowColumn.h>
#include <Xm/ScrolledW.h>
#include <Xm/SelectioB.h>
#include <Xm/ToggleBG.h>
#include <Xmd/Menus.h>
#include <Xmd/Help.h>
#include "filemanager.h"

Widget 
CreateMenuButton(char* name, Widget parent, XtCallbackProc callback,
		 XtPointer data, char mnemonic)
{
  Widget button;
  Arg args[2];
  int n = 0;

  if (mnemonic != 0) {
    XtSetArg(args[n], XmNmnemonic, mnemonic);
    n++;
  }

  button = XmCreatePushButtonGadget(parent, name, args, n);
  XtManageChild(button);

  XtAddCallback(button, XmNactivateCallback, callback, data);
  return(button);
}

Widget 
CreateInterface(char* name, Widget parent)
{
  Widget top, menubar, selected, view, *selected_menu, *view_menu,
         view_pulldown, form, dirOM, where, sw, dirMenu, lroot, 
         help, *help_menu, helpDialog, container, view_sub_menu,
         show_hidden;
  Arg args[30];
  int n;
  Cardinal size;
  XmString tmp, tmp2;
  Pixel white, black, lightblue, gray;
  Display *display;
  Screen *screen;
  XmRendition rend[1];
  XmString headings[10];
  int num_headings;

  /* Create widgets */
  top = XmCreateMainWindow(parent, name, NULL, 0);
  XtManageChild(top);

  /* Goto dialog */
  n = 0;
  tmp = XmStringCreateLocalized("Enter new directory");
  tmp2 = XmStringCreateLocalized("Goto");
  XtSetArg(args[n], XmNselectionLabelString, tmp); n++;
  XtSetArg(args[n], XmNokLabelString, tmp2); n++;
  gotoDialog = XmCreatePromptDialog(top, "gotoDialog", args, n);
  XtAddCallback(gotoDialog, XmNokCallback, gotoCB, NULL);
  XmStringFree(tmp);
  XmStringFree(tmp2);

  /* Menu bar */
  menubar = XmCreateMenuBar(top, "menubar", NULL, 0);
  XtManageChild(menubar);

  XmdCreateMenu(SELECTED_MENU, menubar, &selected_menu, &size);
  XtUnmanageChildren(selected_menu, size);
  XtManageChild(selected_menu[SELECTED_NEW]);
  XtAddCallback(selected_menu[SELECTED_NEW], 
		XmNactivateCallback, 
		(XtCallbackProc) newFolder, NULL);
  XtManageChild(selected_menu[SELECTED_OPEN]);
  XtAddCallback(selected_menu[SELECTED_OPEN], 
		XmNactivateCallback, 
		(XtCallbackProc) manageCB, (XtPointer) gotoDialog);
  XtManageChild(selected_menu[SELECTED_DELETE]);
  XtAddCallback(selected_menu[SELECTED_DELETE], 
		XmNactivateCallback, 
		(XtCallbackProc) deleteItem, NULL);
  XtManageChild(selected_menu[SELECTED_EXIT]);
  XtAddCallback(selected_menu[SELECTED_EXIT],
  		XmNactivateCallback, 
		(XtCallbackProc) quitCB, NULL);

  view_pulldown = XmdCreateMenu(VIEW_MENU, menubar, &view_menu, &size);
  XtUnmanageChildren(view_menu, size);
  XtManageChild(view_menu[VIEW_CHANGE]);

  show_hidden = XmCreateToggleButtonGadget(view_pulldown, "show_hidden",
					   NULL, 0);
  XtManageChild(show_hidden);
  XtAddCallback(show_hidden, XmNvalueChangedCallback, 
		(XtCallbackProc) showHiddenCB, NULL);

  view_sub_menu = XmCreatePulldownMenu(view_pulldown, "viewSubMenu",
				       NULL, 0);
  XtVaSetValues(view_menu[VIEW_CHANGE], XmNsubMenuId, 
		view_sub_menu, NULL, NULL);

  CreateMenuButton("Large", view_sub_menu, 
		   (XtCallbackProc) viewCB, "large", 'L');
  CreateMenuButton("Small", view_sub_menu, 
		   (XtCallbackProc) viewCB, "small", 'S');
  CreateMenuButton("Detail", view_sub_menu, 
		   (XtCallbackProc) viewCB, "detail", 'D');

  helpDialog = XmdCreateHelpDialog(top, "help_manager", NULL, 0);

  XmdCreateMenu(HELP_MENU, menubar, &help_menu, &size);
  XtUnmanageChildren(help_menu, size);
  XtManageChild(help_menu[HELP_OVERVIEW]);
  XtAddCallback(help_menu[HELP_OVERVIEW], XmNactivateCallback,
		(XtCallbackProc) manageCB, (XtPointer) helpDialog);

  /* Main window work area */
  n = 0;
  XtSetArg(args[n], XmNrubberPositioning, False); n++;
  XtSetArg(args[n], XmNresizePolicy, XmRESIZE_ANY); n++;
  form = XmCreateForm(top, "form", args, n);
  XtManageChild(form);

  dirMenu = XmCreatePulldownMenu(top, "dirMenu", NULL, 0);

  /* Create children */
  for(n = 0; n < 9; n++) {
    char buffer[10];
    
    sprintf(buffer, "l%d", n);
    CreateMenuButton(buffer, dirMenu, NULL, NULL, 0);
  }

  /* Create the last child */
  tmp = XmStringCreateLocalized("/");
  n = 0;
  XtSetArg(args[n], XmNlabelString, tmp); n++;
  lroot = XmCreatePushButtonGadget(dirMenu, "lroot", args, n);
  XtAddCallback(lroot, XmNactivateCallback, 
		(XtCallbackProc) readdirCB, (XtPointer) "/");
  XtManageChild(lroot);
  XmStringFree(tmp);

  tmp = XmStringCreateLocalized("Directory");
  n = 0;
  XtSetArg(args[n], XmNlabelString, tmp); n++;
  XtSetArg(args[n], XmNleftAttachment, XmATTACH_FORM); n++;
  XtSetArg(args[n], XmNrightAttachment, XmATTACH_NONE); n++;
  XtSetArg(args[n], XmNtopAttachment, XmATTACH_FORM); n++;
  XtSetArg(args[n], XmNbottomAttachment, XmATTACH_NONE); n++;
  XtSetArg(args[n], XmNresizable, True); n++;
  XtSetArg(args[n], XmNsubMenuId, dirMenu); n++;
  dirOM = XmCreateOptionMenu(form, "dirOM", args, n);
  XtManageChild(dirOM);
  XmStringFree(tmp);

  n = 0;
  XtSetArg(args[n], XmNleftAttachment, XmATTACH_WIDGET); n++;
  XtSetArg(args[n], XmNleftWidget, dirOM); n++;
  XtSetArg(args[n], XmNleftOffset, 30); n++;
  XtSetArg(args[n], XmNtopAttachment, XmATTACH_OPPOSITE_WIDGET); n++;
  XtSetArg(args[n], XmNtopWidget, dirOM); n++;
  XtSetArg(args[n], XmNbottomAttachment, XmATTACH_OPPOSITE_WIDGET); n++;
  XtSetArg(args[n], XmNbottomWidget, dirOM); n++;
  where = XmCreateLabelGadget(form, "Where", args, n);
  XtManageChild(where);

  n = 0;
  XtSetArg(args[n], XmNwidth, 450); n++;
  XtSetArg(args[n], XmNheight, 350); n++;
  XtSetArg(args[n], XmNleftAttachment, XmATTACH_FORM); n++;
  XtSetArg(args[n], XmNrightAttachment, XmATTACH_FORM); n++;
  XtSetArg(args[n], XmNtopAttachment, XmATTACH_WIDGET); n++;
  XtSetArg(args[n], XmNtopWidget, dirOM); n++;
  XtSetArg(args[n], XmNtopOffset, 5); n++;
  XtSetArg(args[n], XmNbottomAttachment, XmATTACH_FORM); n++;
  XtSetArg(args[n], XmNresizable, True); n++;
  XtSetArg(args[n], XmNleftOffset, 5); n++;
  XtSetArg(args[n], XmNrightOffset, 5); n++;
  XtSetArg(args[n], XmNbottomOffset, 5); n++;
  XtSetArg(args[n], XmNscrollingPolicy, XmAUTOMATIC); n++;
  XtSetArg(args[n], XmNscrollBarDisplayPolicy, XmSTATIC); n++;
  XtSetArg(args[n], XmNscrolledWindowMarginWidth, 5); n++;
  XtSetArg(args[n], XmNscrolledWindowMarginHeight, 5); n++;
  sw = XmCreateScrolledWindow(form, "sw", args, n);
  XtManageChild(sw);

  num_headings = 0;
  headings[num_headings++] = XmStringCreateLocalized("Filename");
  headings[num_headings++] = XmStringCreateLocalized("Owner");
  headings[num_headings++] = XmStringCreateLocalized("Permissions");
  headings[num_headings++] = XmStringCreateLocalized("Size");

  n = 0;
  XtSetArg(args[n], XmNspatialIncludeModel, XmAPPEND); n++;
  XtSetArg(args[n], XmNspatialStyle, XmGRID); n++;
  XtSetArg(args[n], XmNspatialResizeModel, XmGROW_MINOR); n++;
  XtSetArg(args[n], XmNspatialSnapModel, XmCENTER); n++;
  XtSetArg(args[n], XmNdetailColumnHeading, headings); n++;
  XtSetArg(args[n], XmNdetailColumnHeadingCount, num_headings); n++;
  container = XmCreateContainer(sw, "container", args, n);
  XtManageChild(container);
  XtAddCallback(container, XmNdefaultActionCallback, 
		(XtCallbackProc) selectCB, NULL);
  XtAddCallback(container, XmNoutlineChangedCallback, 
		(XtCallbackProc) outlineCB, NULL);

  for(n = 0; n < num_headings; n++)
    XmStringFree(headings[n]);

  return(top);
}

/* $TOG: readdir.c /main/7 1997/03/31 13:53:34 dbl $ */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pwd.h>
#include <Xm/Xm.h>           /* Motif Toolkit */
#include <Mrm/MrmPublic.h>   /* Mrm */
#include <Xm/Container.h>
#include <Xm/IconG.h>
#include "filemanager.h"

Pixmap fileIcon, dirIcon, fileMask, dirMask, execIcon, execMask,
       noneIcon, noneMask;
Pixmap s_fileIcon, s_dirIcon, s_fileMask, s_dirMask, 
  s_execIcon, s_execMask, s_noneIcon, s_noneMask;

#define NDIRLEVELS 9
int ndirLabel = NDIRLEVELS;
Widget *dirLabel;
char* paths[NDIRLEVELS];
Boolean showHidden = False;

char *currentdir = NULL;

int  validFI = 0; 
int  maxFI = 0;
FileInfoRec *FI = NULL;

int  maxIcons = 0;
int  validIcons = 0;
WidgetList IconGadgets = NULL;

static void doOptionMenu();

static void getIcons(int ind, Pixmap *icon, Pixmap *mask, 
		     Pixmap *sicon, Pixmap *smask);
static int GetNewEntry(int, char*, struct dirent*);
static FileInfoRec *GetInfoFromWidget(Widget);
typedef int (*qsProc)(const void *, const void *);

static int 
qsCompare(void *xx, void *yy)
{
  FileInfoRec *x = (FileInfoRec *) xx;
  FileInfoRec *y = (FileInfoRec *) yy;
  return(strcmp(x -> name, y -> name));
}

static void SortChildren()
{
  int i;

  if (validFI == 0) return;

  qsort(FI, validFI, sizeof(FileInfoRec), (qsProc) qsCompare);

  /* Reorder the list of gadgets */
  for(i = 0; i < validFI; i++) 
    IconGadgets[i] = FI[i].icon;
}


/* Determine real full pathname */
/* This code is UNIX pathname dependent */
char* 
expandPath(char *dirname) 
{
  char buf[1024];
  Boolean parentdir = False;
  Boolean curdir = False;
  Boolean isroot = False;
  char *dir;
  int length;

  if (strcmp(dirname, "/") == 0) isroot = True;
  if (strncmp(dirname, "..", 2) == 0) parentdir = True;
  if (! parentdir && strncmp(dirname, ".", 1) == 0) curdir = True;

  if (! isroot && 
      (curdir || parentdir)) {
    char *n;
    char *partial;

    /* Move currentdir info into a buffer for manipulation */
    if (currentdir == NULL) {
      getcwd(buf, 1024);
    } else {
      strcpy(buf, currentdir);
    }

    dir = buf;
    length = strlen(dir);
    if (length > 2 && dir[length] == '/') dir[length] = 0;

    if (parentdir) {
      n = strrchr(dir, '/');
      if (n != NULL) n = strrchr(dir, '/');
      if (n != NULL) *n = 0; /* truncate */
    }

    /* Look for ../ or ./ at the beginning of the string */
    partial = strchr(dirname, '/');
    if (partial != NULL) {
      strcat(dir, "/");
      strcat(dir, partial);
    } 

    /* If we are empty here,  then we started with ../ */
    if (dir[0] == 0) strcpy(dir, "/");

    return(XtNewString(dir));
  } else {
    return(XtNewString(dirname));
  }
}

void 
readdirCB(Widget widget, char* dirname, XmAnyCallbackStruct *callback_data)
{
  read_directory(NULL, dirname);
}

void 
outlineCB(Widget widget, char* subdirname, 
	  XmContainerOutlineCallbackStruct *callback_data)
{
  if (callback_data -> new_outline_state == True) {
    Widget parent = callback_data -> item;
    char *path;
    int ind;
    FileInfoRec *f = GetInfoFromWidget(parent);

    if (! f -> dirRead) {
      /* Create full pathname.  More complex as we must go find the
	 all the parent's names first */
      Widget grandparent;
      char *names[32]; /* They had better not go more than 32 levels
			  deep in the view ! */
      char buf[256];
      int level;

      f->dirRead = True;
      level = 0;
      /* First go get all the strings */
      XtVaGetValues(parent, XmNentryParent, &grandparent, NULL, NULL);
      names[level++] = f -> name;
      for(; grandparent != (Widget) NULL; level++) {
	FileInfoRec *f = GetInfoFromWidget(grandparent);
	if (f) names[level] = f -> name;
	/* Get the next parent */
	XtVaGetValues(grandparent, XmNentryParent, &grandparent, NULL, NULL);
      }

      /* Now assemble the names into a string */
      strcpy(buf, "");
      for(level = level - 1; level >= 0; level--) {
	strcat(buf, "/"); /* Unix dependent */
	strcat(buf, names[level]);
      }
      path = fullpath(buf);
      if (path != NULL) {
	read_directory(parent, path);
	XtFree(path);
      }
    }
  }
}

void
read_directory(Widget parent, char* dirname)
{
  DIR *thisdir;
  struct dirent *info;
  int i;
  char *tempdir = NULL;

  if (parent == NULL) {
    XtFree(currentdir);
    currentdir = expandPath(dirname);
    thisdir = opendir(currentdir);
  } else {
    tempdir = expandPath(dirname);
    thisdir = opendir(tempdir);
  }

  if (thisdir == NULL) return;

  if (parent == NULL) {
    struct stat buf;

    /* Unmanage all iconGadgets */
    XtUnmanageChildren(IconGadgets,validFI);

    /* Free old namestrings */
    for(i = 0; i < validFI; i++) XtFree(FI[i].name);
    validFI = 0; /* Reset */

    /* Update last read time for current directory */
    stat(currentdir, &buf);
    ltm = buf.st_ctime;    
  }

  while((info = readdir(thisdir))) {
    Boolean is_dot, is_dotdot;

    is_dot = strcmp(info -> d_name, ".") == 0;
    is_dotdot = strcmp(info -> d_name, "..") == 0;

    /* We always process non-dot name files.  But if they start
       with a dot,  we process them only if showHidden is true,
       or they are one of dot or dot-dot */
    if ((is_dot && parent == (Widget) NULL) ||
	showHidden || is_dotdot || (! (info -> d_name[0] == '.'))) {
      validFI = process_single_entry(parent, tempdir, validFI, info);
      validFI++;
    }
  }

  /* Sort children and relayout container */
  SortChildren();
  XmContainerReorder(fileviewer, IconGadgets, validFI);

  /* Manage all valid iconGadgets */
  XtManageChildren(IconGadgets,validFI);

  XmContainerRelayout(fileviewer);

  if (parent == NULL) {
    doOptionMenu();
  }

  /* Restrict geometry of Container */
  fixViewerSize(XtParent(fileviewer), NULL, NULL);

  closedir(thisdir);

  XtFree(tempdir);
}

int 
process_single_entry(Widget parent, char* dir, int ind,
		     struct dirent *info)
{
  Pixmap icon, mask;
  Pixmap s_icon, s_mask;
  char buf[64];
  int status;
  int i;
  float size;
  XmString details[10];
  XmString stemp;
  int num_details;
  mode_t mode;
  struct passwd *user;

  if (parent)
    status = GetNewEntry(ind, dir, info);
  else
    status = GetNewEntry(ind, currentdir, info);
  
  icon = mask = s_icon = s_mask = XmUNSPECIFIED_PIXMAP;

  if (status != -1)
    getIcons(ind, &icon, &mask, &s_icon, &s_mask);
  else
    getIcons(-1, &icon, &mask, &s_icon, &s_mask);
  
  num_details = 0;
  /* Owner */
  if (status != -1)
    user = getpwuid(FI[ind].statbuf.st_uid);
  else
    user = NULL;
  
  if (user != NULL)
    details[num_details++] = XmStringCreateLocalized(user->pw_name);
  else {
    sprintf(buf, "%d", FI[ind].statbuf.st_uid);
    details[num_details++] = XmStringCreateLocalized(buf);
  }
  
  /* Permissions */
  if (status != -1)
    mode = FI[ind].statbuf.st_mode;
  else
    mode = 0;
  sprintf(buf, "%c%c%c,%c%c%c,%c%c%c", 
	  (S_IRUSR & mode) ? 'r' : '-',
	  (S_IWUSR & mode) ? 'w' : '-',
	  (S_IXUSR & mode) ? 'x' : '-',
	  (S_IRGRP & mode) ? 'r' : '-',
	  (S_IWGRP & mode) ? 'w' : '-',
	  (S_IXGRP & mode) ? 'x' : '-',
	  (S_IROTH & mode) ? 'r' : '-',
	  (S_IWOTH & mode) ? 'w' : '-',
	  (S_IXOTH & mode) ? 'x' : '-');
  details[num_details++] = XmStringCreateLocalized(buf); 
  /* Size */
  if (status != -1)
    size = FI[ind].statbuf.st_size;
  else
    size = 0;

  if (size < 1000.0)
    sprintf(buf, "%-d", (int) size);
  else if (size < 1.0e6)
    sprintf(buf, "%-.2fK", size/1.0e3);
  else
    sprintf(buf, "%-.2fM", size/1.0e6);
  details[num_details++] = XmStringCreateLocalized(buf); 

  stemp = XmStringCreateLocalized(info -> d_name);
  XtVaSetValues(IconGadgets[ind], 
		XmNlabelString, stemp, 
		XmNlargeIconPixmap, icon, 
		XmNlargeIconMask, mask,
		XmNsmallIconPixmap, s_icon, 
		XmNsmallIconMask, s_mask,
		XmNdetail, details,
		XmNentryParent, parent,
		XmNoutlineState, XmCOLLAPSED,
		XmNdetailCount, num_details,
		NULL, NULL);
  FI[ind].icon = IconGadgets[ind];

  XmStringFree(stemp);
  
  if (S_ISDIR(FI[ind].statbuf.st_mode) &&
      strcmp(info -> d_name, ".") != 0 &&
      strcmp(info -> d_name, "..") != 0) {
    ind++;
    FI[ind].dirRead = False;
    /* Create "." child */
    GetNewEntry(ind, currentdir, info);
    stemp = XmStringCreateLocalized(".");
    XtVaSetValues(IconGadgets[ind], 
		  XmNlabelString, stemp, 
		  XmNlargeIconPixmap, icon, 
		  XmNlargeIconMask, mask,
		  XmNsmallIconPixmap, s_icon, 
		  XmNsmallIconMask, s_mask,
		  XmNdetail, details,
		  XmNdetailCount, num_details,
		  XmNentryParent, IconGadgets[ind-1],
		  NULL, NULL);
    FI[ind].icon = IconGadgets[ind];
    XmStringFree(stemp);
  }

  for(i = 0; i < num_details; i++)
    XmStringFree(details[i]);

  return(ind);
}     


static int
GetNewEntry(int vFI, char* dir, struct dirent *info)
{
  char buf[256];
  int status;

  if (vFI >= maxFI) {
    int newsize;
    if (maxFI == 0) {
      newsize = 64;
      FI = (FileInfoRec *) XtMalloc(newsize * sizeof(FileInfoRec));
    } else {
      newsize = maxFI * 2;
      FI = (FileInfoRec *) XtRealloc((XtPointer) FI,
				     newsize * sizeof(FileInfoRec));
    }
    maxFI = newsize;
  }

  FI[vFI].name = XtNewString(info -> d_name);
  /* More UNIX specific code */
  strcpy(buf, dir);
  strcat(buf, "/");
  strcat(buf, info -> d_name);
  status = stat(buf, &FI[vFI].statbuf);

  if (vFI >= maxIcons) {
    int newsize;
    if (maxIcons == 0) {
      newsize = 64;
      IconGadgets = (Widget *) XtMalloc(sizeof(Widget) * newsize);
    } else {
      newsize = maxIcons * 2;
      IconGadgets = (Widget *) XtRealloc((XtPointer) IconGadgets, 
					 sizeof(Widget) * newsize);
    }
    maxIcons = newsize;
  }
  
  if (vFI >= validIcons) {
    IconGadgets[vFI] = XmCreateIconGadget(fileviewer, "IG", NULL, 0);
    validIcons++;
  }

  return status;
}

static char*
find_suffix(char *filename)
{
  int i = strlen(filename);

  while(i > 0 && filename[i] != '.') i--;

  if (filename[i] == '.')
    return(&filename[i + 1]);
  else 
    return(filename);
}

static void readIcon(str, icon, mask, fg, bg)
     char *str;
     Pixmap *icon;
     Pixmap *mask;
     Pixel fg, bg;
{
  if (str != NULL) {
    char msk[256];

    if (strcmp(find_suffix(str), "xpm") == 0) {
      int len = strlen(str);
      strncpy(msk, str, len - 4);
      msk[len - 4] = 0;
      strcat(msk, "_m.xpm");
    } else {
      strcpy(msk, str);
      strcat(msk, "_m");
    }

    *icon = XmGetPixmap(XtScreen(toplevel), str, fg, bg);
    *mask = XmGetPixmapByDepth(XtScreen(toplevel), msk,
                               WhitePixelOfScreen(XtScreen(toplevel)),
                               BlackPixelOfScreen(XtScreen(toplevel)), 1);
  }
}

static void 
getIcons(int ind, Pixmap *icon, Pixmap *mask, Pixmap *sicon, Pixmap *smask)
{
  Boolean	isdir;
  Boolean	isexec;
  Boolean	canRead;
  FileInfoRec *info = &FI[ind];
  mode_t	mode;
  XrmQuark	path[10];
  XrmQuark	classes[10];
  XrmDatabase	db = XtScreenDatabase(XtScreenOfObject(toplevel));
  XrmValue	value;
  XrmRepresentation type;
  Pixel		fg, bg;
  char		*str;
  char		*default_type = NULL;

  XtVaGetValues(fileviewer, XmNforeground, &bg, XmNbackground, &fg,
		NULL, NULL);

  /* First try the resource database,  then use the fallbacks below */
  classes[0] = app_class_quark;
  classes[1] = XrmStringToQuark("Types");
  classes[2] = XrmStringToQuark("Any");
  classes[3] = XrmStringToQuark("Icon");
  classes[4] = NULLQUARK;
  path[4] = NULLQUARK;

  path[0] = app_quark;
  path[1] = XrmStringToQuark("types");

  if (ind < 0) {
    default_type = "default_none";
  } else {
    path[2] = XrmStringToQuark(find_suffix(info -> name)); 
    path[3] = XrmStringToQuark("largeIcon");
    XrmQGetResource(db, path, classes, &type, &value);
    str = (char*) value.addr;
    readIcon(str, icon, mask, fg, bg);

    path[3] = XrmStringToQuark("smallIcon");
    XrmQGetResource(db, path, classes, &type, &value);
    str = (char*) value.addr;
    readIcon(str, sicon, smask, fg, bg);

    if (*icon != XmUNSPECIFIED_PIXMAP && 
	*sicon != XmUNSPECIFIED_PIXMAP) return;
  }

  isdir = S_ISDIR(info -> statbuf.st_mode);
  mode = info -> statbuf.st_mode;
  isexec = (mode & S_IXUSR) | (mode & S_IXGRP) | (mode & S_IXOTH);

  /* Defaults */
  if (default_type != NULL) {
    path[2] = XrmStringToQuark(default_type);
  } else if (isdir) {
    path[2] = XrmStringToQuark("default_dir");
  } else if (isexec) {
    path[2] = XrmStringToQuark("default_exec");
  } else {
    path[2] = XrmStringToQuark("default_file");
  }

  if (*icon == XmUNSPECIFIED_PIXMAP) {
    path[3] = XrmStringToQuark("largeIcon");
    XrmQGetResource(db, path, classes, &type, &value);
    str = (char*) value.addr;
    readIcon(str, icon, mask, fg, bg);
  }

  if (*sicon == XmUNSPECIFIED_PIXMAP) {
    path[3] = XrmStringToQuark("smallIcon");
    XrmQGetResource(db, path, classes, &type, &value);
    str = (char*) value.addr;
    readIcon(str, sicon, smask, fg, bg);
  }
}


/* Setup Option Menu */
/* Break off components and stuff into the pushbuttons */
/* This code is entirely UNIX pathname dependent */

static void doOptionMenu()
{
  int i;
  XmString stemp;
  char *c = currentdir;
  Widget memWidget;

  XtUnmanageChildren(dirLabel, ndirLabel);

  for (i = 0; i < ndirLabel; i++) {
    XtFree(paths[i]);
    paths[i] = NULL;
  }

  i = 0;

  if (*c == '/') c++; /* Pointing at dir sep */
  while(i < ndirLabel &&
	*c != 0) {
    char buf[128];
    int n;
    int rind;
    int span;

    /* Copy dir name */
    for(n = 0; n < 128 && c[n] != '/' && c[n] != 0; n++)
      buf[n] = c[n];
    buf[n] = 0;
    c = &c[n];
    
    rind = ndirLabel - i - 1;
    if (rind < 0) break;

    /* Copy full path */
    span = c - currentdir + 1;
    paths[i] = XtMalloc(span + 1);
    strncpy(paths[i], currentdir, span);
    paths[i][span] = 0; /* Null terminate */
    stemp = XmStringCreateLocalized(buf);
    XtVaSetValues(dirLabel[rind], 
		  XmNlabelString, stemp, 
		  NULL, NULL);
    XmStringFree(stemp);
    XtManageChild(dirLabel[rind]);
    XtRemoveAllCallbacks(dirLabel[rind], XmNactivateCallback);
    XtAddCallback(dirLabel[rind], XmNactivateCallback, 
		  (XtCallbackProc) readdirCB, paths[i]);
    i++;
    while (*c == '/' && *c != 0)
      c++; /* Pointing at dir sep */
  }
  if (ndirLabel - i >= 0 && i != 0)
    memWidget = dirLabel[ndirLabel - i];
  else if (i == 0)
    memWidget = XtNameToWidget(mainW, "*lroot");
  else
    memWidget = dirLabel[0];
  XtVaSetValues(dirOM, XmNmenuHistory, memWidget, NULL, NULL);

  /* Reset label */
  stemp = XmStringCreateLocalized(currentdir);
  XtVaSetValues(displayLabel, XmNlabelString, stemp, NULL, NULL);
}

void 
selectCB(Widget w, XtPointer ignore, XtPointer cb)
{
  XmContainerSelectCallbackStruct *cbstruct = 
    (XmContainerSelectCallbackStruct *) cb;
  char *temp;
  Boolean found = False;
  Widget target = cbstruct -> selected_items[0];
  mode_t mode;
  FileInfoRec *f;

  if (cbstruct -> selected_item_count != 1) return;

  f = GetInfoFromWidget(target);

  if (! f) return;

  mode = f -> statbuf.st_mode;
  
  if (S_ISDIR(mode)) {
    /* For directories,  navigate downward */
    if (strcmp(f -> name, "..") == 0 ||
	strcmp(f -> name, ".") == 0)
      temp = expandPath(f -> name);
    else
      temp = fullpath(f -> name);
    readdirCB(w, temp, NULL);
    XtFree(temp);
  } else {
    XrmDatabase db = XtScreenDatabase(XtScreen(toplevel));
    XrmQuark	classes[10];
    XrmQuark	path[10];
    XrmRepresentation type;
    XrmValue	value;
    char	*str;

    /* First try the resource database,  then use the fallbacks below */
    classes[0] = app_class_quark;
    classes[1] = XrmStringToQuark("Types");
    classes[2] = XrmStringToQuark("Any");
    classes[3] = XrmStringToQuark("Action");
    classes[4] = NULLQUARK;

    path[0] = app_quark;
    path[1] = XrmStringToQuark("types");
    path[2] = XrmStringToQuark(find_suffix(f -> name)); 
    path[3] = XrmStringToQuark("action");
    path[4] = NULLQUARK;

    XrmQGetResource(db, path, classes, &type, &value);
    str = (char*) value.addr;
    if (str == NULL) {
      if ((mode & S_IXUSR) | (mode & S_IXGRP) | (mode & S_IXOTH)) {
	path[2] = XrmStringToQuark("default_exec");
      } else {
	path[2] = XrmStringToQuark("default_file");
      }
      XrmQGetResource(db, path, classes, &type, &value);
      str = (char*) value.addr;
    }
    if (str != NULL) {
      char *pathname = fullpath(f -> name);
      char buf[256];
      sprintf(buf, str, pathname);
      strcat(buf, " &");
      system(buf);
      XtFree(pathname);
    }
  }
}

char *
getPathFromIcon(Widget w)
{
  FileInfoRec *f = GetInfoFromWidget(w);

  if (f)
    return(fullpath(f -> name));
  else
    return(NULL);
}

static FileInfoRec *
GetInfoFromWidget(Widget w)
{
  Boolean found = False;
  int ind;

  for(ind = 0; ind < validFI; ind++) {
    if (IconGadgets[ind] == w) {
      found = True;
      break;
    }
  }

  if (found)
    return(&FI[ind]);
  else
    return(NULL);
}