
start /WAIT /B taskkill /f /im vcxsrv.exe
start "" "C:\Program Files\VcXsrv\vcxsrv.exe" -multiwindow -clipboard -wgl -ac

wt ^
	ssh -Y momav@ground; ^
split-pane -V ^
	ssh momav@momav -J momav@ground; ^
move-focus left; split-pane -H ^
	wsl -d momav; ^
move-focus right; split-pane -H ^
	ssh momav@momav -J momav@ground