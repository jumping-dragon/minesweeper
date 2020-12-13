/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.40                          *
*        Compiled Jun 22 2017, 10:13:26                              *
*        (c) 2017 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
// USER END

#include "main.h"
#include "DIALOG.h"
#include "GameWindowDLG.h"
#include <time.h>

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, 0, 480, 272, 0, 0x0, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_0, 13, 13, 39, 26, 0, 0x2, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_1, 409, 13, 51, 24, 0, 0x5, 0 },
  { TEXT_CreateIndirect, "Text_state", ID_TEXT_0, 190, 15, 80, 27, 0, 0x64, 0 },
  //1
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_0, 45, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_1, 45, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_2, 45, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_3, 45, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_4, 45, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_5, 45, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_6, 45, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_7, 45, 220, 25, 25, 0, 0, 0 },
  //2
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_8, 70, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_9, 70, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_10, 70, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_11, 70, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_12, 70, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_13, 70, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_14, 70, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_15, 70, 220, 25, 25, 0, 0, 0 },
  //3
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_16, 95, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_17, 95, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_18, 95, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_19, 95, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_20, 95, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_21, 95, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_22, 95, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_23, 95, 220, 25, 25, 0, 0, 0 },
  //4
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_24, 120, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_25, 120, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_26, 120, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_27, 120, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_28, 120, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_29, 120, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_30, 120, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_31, 120, 220, 25, 25, 0, 0, 0 },
  //5
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_32, 145, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_33, 145, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_34, 145, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_35, 145, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_36, 145, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_37, 145, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_38, 145, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_39, 145, 220, 25, 25, 0, 0, 0 },
  //6
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_40, 170, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_41, 170, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_42, 170, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_43, 170, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_44, 170, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_45, 170, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_46, 170, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_47, 170, 220, 25, 25, 0, 0, 0 },
  //7
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_48, 195, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_49, 195, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_50, 195, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_51, 195, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_52, 195, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_53, 195, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_54, 195, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_55, 195, 220, 25, 25, 0, 0, 0 },
  //8
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_56, 220, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_57, 220, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_58, 220, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_59, 220, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_60, 220, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_61, 220, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_62, 220, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_63, 220, 220, 25, 25, 0, 0, 0 },
  //9
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_64, 245, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_65, 245, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_66, 245, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_67, 245, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_68, 245, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_69, 245, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_70, 245, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_71, 245, 220, 25, 25, 0, 0, 0 },
  //10
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_72, 270, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_73, 270, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_74, 270, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_75, 270, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_76, 270, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_77, 270, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_78, 270, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_79, 270, 220, 25, 25, 0, 0, 0 },
  //11
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_80, 295, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_81, 295, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_82, 295, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_83, 295, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_84, 295, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_85, 295, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_86, 295, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_87, 295, 220, 25, 25, 0, 0, 0 },
  //12
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_88, 320, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_89, 320, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_90, 320, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_91, 320, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_92, 320, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_93, 320, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_94, 320, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_95, 320, 220, 25, 25, 0, 0, 0 },
  //13
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_96, 345, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_97, 345, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_98, 345, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_99, 345, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_100, 345, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_101, 345, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_102, 345, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_103, 345, 220, 25, 25, 0, 0, 0 },
  //14
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_104, 370, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_105, 370, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_106, 370, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_107, 370, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_108, 370, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_109, 370, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_110, 370, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_111, 370, 220, 25, 25, 0, 0, 0 },
  //15
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_112, 395, 45, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_113, 395, 70, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_114, 395, 95, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_115, 395, 120, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_116, 395, 145, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_117, 395, 170, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_118, 395, 195, 25, 25, 0, 0, 0 },
    { IMAGE_CreateIndirect, "Image", ID_IMAGE_119, 395, 220, 25, 25, 0, 0, 0 },
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _GetImageById
*/
static const void * _GetImageById(U32 Id, U32 * pSize) {
  switch (Id) {
  case ID_WALL:
    *pSize = sizeof(ID_WALL);
    return (const void *)WALL;
  case ID_NUMBER1:
    *pSize = sizeof(ID_NUMBER1);
    return (const void *)NUMBER1;
  case ID_NUMBER2:
    *pSize = sizeof(ID_NUMBER2);
    return (const void *)NUMBER2;
  case ID_NUMBER3:
    *pSize = sizeof(ID_NUMBER3);
    return (const void *)NUMBER3;
  case ID_NUMBER4:
    *pSize = sizeof(ID_NUMBER4);
    return (const void *)NUMBER4;
  case ID_NUMBER5:
    *pSize = sizeof(ID_NUMBER6);
    return (const void *)NUMBER6;
  case ID_NUMBER6:
    *pSize = sizeof(ID_NUMBER6);
    return (const void *)NUMBER6;
  case ID_NUMBER7:
    *pSize = sizeof(ID_NUMBER7);
    return (const void *)NUMBER7;
  case ID_NUMBER8:
    *pSize = sizeof(ID_NUMBER8);
    return (const void *)NUMBER8;
  case ID_NUMBER9:
    *pSize = sizeof(ID_NUMBER9);
    return (const void *)NUMBER9;
  case ID_BOMB:
    *pSize = sizeof(ID_BOMB);
    return (const void *)BOMB;
  case ID_BLANK:
    *pSize = sizeof(ID_BLANK);
    return (const void *)BLANK;
  }
  return NULL;
}

// USER START (Optionally insert additional static code)
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  const void * pData;
  WM_HWIN      hItem;
  U32          FileSize;
  int          NCode;
  int          Id;
  char bomb[12];
  static char buffer[300] = "00:00\r\n";

  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Edit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
    if(pMsg->Data.v){
      sprintf(bomb,"%d",pMsg->Data.v);
      EDIT_SetText(hItem, bomb);
    }
    else{
      EDIT_SetText(hItem, "0");
    }
    EDIT_SetFont(hItem, GUI_FONT_16B_ASCII);
    EDIT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    EDIT_SetFocussable(hItem, EDIT_CI_DISABLED);
    //
    // Initialization of 'Edit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1);
    EDIT_SetText(hItem, buffer);
    EDIT_SetFont(hItem, GUI_FONT_16B_ASCII);
    EDIT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    EDIT_SetFocussable(hItem, EDIT_CI_DISABLED);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_24B_ASCII);
    TEXT_SetText(hItem, "WIN");
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00BD1D00));
    //
    // Initialization of 'Image'
    //
    for(int x = ID_IMAGE_0; x <= ID_IMAGE_119; x++){
      hItem = WM_GetDialogItem(pMsg->hWin, x);
      pData = _GetImageById(ID_NUMBER1, &FileSize);
      IMAGE_SetBMP(hItem, pData, FileSize);
    }

    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_EDIT_0: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_1: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    case ID_IMAGE_0:
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        printf("test\r\n");
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  case 300:
    sprintf(buffer, "%d%d:%d%d",pMsg->Data.v/600, pMsg->Data.v/60, pMsg->Data.v%60/10, pMsg->Data.v%10 );
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1);
    EDIT_SetText(hItem, buffer);
    break;
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateWindow2
*/

WM_HWIN CreateWindow2(int bombAmount);
WM_HWIN CreateWindow2(int bombAmount) {
  WM_HWIN hWin;
  WM_MESSAGE Initial;
  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  printf("%d",bombAmount);
  Initial.MsgId = WM_INIT_DIALOG;
  Initial.Data.v = bombAmount;
  WM_SendMessage(hWin,&Initial);
  // struct WM_MESSAGE {
        // int MsgId;            /* type of message */
        // WM_HWIN hWin;         /* Destination window */
        // WM_HWIN hWinSrc;      /* Source window  */
        // union {
        //   const void * p;            /* Some messages need more info ... Pointer is declared "const" because some systems (M16C) have 4 byte const, byte 2 byte default ptrs */
        //   int v;
        //   GUI_COLOR Color;
        // } Data;
        // };
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
