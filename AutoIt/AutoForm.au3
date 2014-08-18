; -*- coding: utf-8 -*-
#include <ButtonConstants.au3>
#include <EditConstants.au3>
#include <GUIConstantsEx.au3>
#include <GuiEdit.au3>
#include <WindowsConstants.au3>
#include "CommInterface.au3"
#include "ComPortNo.au3"
;Local Const $iPort = 3

Local $hFile = _CommAPI_OpenCOMPort($iPort, 9600, 0, 8, 0)
If @error Then
	MsgBox($MB_OK, "Error", "Failed to open COM" & $iPort & "." & @error)
	Exit
EndIf

#Region ### START Koda GUI section ### Form=c:\stm32cube\servolifter\autoit\autoform.kxf
$Form1_1 = GUICreate("ServoLifter demo", 362, 353, 693, 103)
GUISetFont(9, 400, 0, "ＭＳ Ｐゴシック")
$Button1 = GUICtrlCreateButton("PUT A", 16, 16, 75, 25)
$Button2 = GUICtrlCreateButton("PUT B", 16, 48, 75, 25)
$Button3 = GUICtrlCreateButton("PUT C", 16, 80, 75, 25)
$Button4 = GUICtrlCreateButton("PUT D", 16, 112, 75, 25)
$Button5 = GUICtrlCreateButton("PUT R", 16, 144, 75, 25)
$Button6 = GUICtrlCreateButton("TAKE", 16, 192, 75, 25)
$Button7 = GUICtrlCreateButton("CLEAR", 16, 240, 75, 25)
$Button8 = GUICtrlCreateButton("NEUTRAL", 16, 272, 75, 25)
$Button9 = GUICtrlCreateButton("Exit", 16, 312, 75, 25)
$Edit1 = GUICtrlCreateEdit("", 112, 16, 225, 321)
GUICtrlSetData(-1, "")
GUISetState(@SW_SHOW)
#EndRegion ### END Koda GUI section ###

While 1
	$nMsg = GUIGetMsg()
	Switch $nMsg
		Case $Button1
			_CommAPI_TransmitString($hFile, "PUT A" & @CR)
			ReceiveUntilOK($hFile)
		Case $Button2
			_CommAPI_TransmitString($hFile, "PUT B" & @CR)
			ReceiveUntilOK($hFile)
		Case $Button3
			_CommAPI_TransmitString($hFile, "PUT C" & @CR)
			ReceiveUntilOK($hFile)
		Case $Button4
			_CommAPI_TransmitString($hFile, "PUT D" & @CR)
			ReceiveUntilOK($hFile)
		Case $Button5
			_CommAPI_TransmitString($hFile, "PUT R" & @CR)
			ReceiveUntilOK($hFile)
		Case $Button6
			_CommAPI_TransmitString($hFile, "TAKE" & @CR)
			ReceiveUntilOK($hFile)
		Case $Button7
			_CommAPI_TransmitString($hFile, "CLEAR" & @CR)
			ReceiveUntilOK($hFile)
		Case $Button8
			_CommAPI_TransmitString($hFile, "NEUTRAL" & @CR)
			ReceiveUntilOK($hFile)
		Case $Button9
			_CommAPI_ClosePort($hFile)
			Exit
		Case $GUI_EVENT_CLOSE
			ConsoleWrite('@@ Debug(' & @ScriptLineNumber & ') : $GUI_EVENT_CLOSE = ' & $GUI_EVENT_CLOSE & @CRLF & '>Error code: ' & @error & @CRLF) ;### Debug Console
			_CommAPI_ClosePort($hFile)
			Exit

	EndSwitch
WEnd

Func ReceiveUntilOK($iFile)
	Local $Received = ""
	Local $sResult
	Local $retry = 20
	While True
		$sResult = _CommAPI_ReceiveString($hFile, 1000)
		If @error Then
			If @error = -1 Then
				$retry = $retry - 1
				If $retry > 0 then
					ContinueLoop
				EndIf
				_GUICtrlEdit_AppendText($Edit1, "Retry timeout" & @CRLF)
				ExitLoop
			Else
				Return SetError(@error, @ScriptLineNumber, $sResult)
			EndIf
		EndIf
		$Received = $Received & $sResult
		_GUICtrlEdit_AppendText($Edit1, $sResult)
		If StringInStr($sResult, "OK") Then
			ExitLoop
		EndIf
	WEnd
	return $sResult
EndFunc
