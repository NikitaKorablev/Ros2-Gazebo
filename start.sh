#!/bin/bash
# Запуск виртуального X-сервера
Xvfb :1 -screen 0 1920x1080x24 &
sleep 2

# Запуск VNC сервера
x11vnc -display :1 -nopw -forever -shared -rfbport 5900 &
sleep 2

# Запуск noVNC (веб-интерфейс)
/usr/share/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &

# Оставляем контейнер запущенным
tail -f /dev/null