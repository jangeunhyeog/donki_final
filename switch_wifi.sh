#!/bin/bash
# switch_wifi.sh
# Usage: ./switch_wifi.sh [SSID] [PASSWORD]

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "사용법: ./switch_wifi.sh [와이파이_이름] [비밀번호]"
    echo "예시: ./switch_wifi.sh my_hotspot 12345678"
    exit 1
fi

SSID=$1
PASSWORD=$2

echo "와이파이 목록을 새로고침합니다..."
sudo nmcli device wifi rescan
sleep 2

echo "'$SSID' 에 연결을 시도합니다..."
sudo nmcli device wifi connect "$SSID" password "$PASSWORD"

if [ $? -eq 0 ]; then
    echo "✅ 성공적으로 '$SSID'에 연결되었습니다!"
    echo "현재 할당된 IP 주소:"
    ip -4 addr show wlp1s0 | grep inet | awk '{print $2}'
else
    echo "❌ 연결에 실패했습니다. 와이파이 이름과 비밀번호를 다시 확인해주세요."
fi
