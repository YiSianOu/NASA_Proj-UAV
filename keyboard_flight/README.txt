ROS 2 Humble 多旋翼鍵盤控制（PX4 v1.17，pymavlink）
==================================================
把本目錄整份放在 Linux 主機或容器中。三個終端使用相同 ROS_DOMAIN_ID，
都先進入本目錄並執行 source /opt/ros/humble/setup.bash。
執行環境需有 rclpy、std_msgs、pymavlink；繪圖端另需 Matplotlib 與可用的圖形桌面。
腳本直接使用 python3 啟動，控制節點需能讀寫指定的 MAVLink 連線。

終端 1（只有此程式連線到 PX4）
    python3 main.py --ros-args -p port:=/dev/ttyUSB0 -p baud:=57600
    已有 PX4 SITL 對本機 UDP 14540 送資料時，改用以下指令：
    python3 main.py --ros-args -p port:=udpin:0.0.0.0:14540

終端 2（狀態與三軸曲線）
    python3 px4_states_monitor.py

終端 3（鍵盤）
    python3 mode_select.py

操作
----
1. 輸入 takeoff。地面且未解鎖時，控制程式先請求 PX4 自動起飛模式，收到
   模式回報後送正常解鎖指令，由 PX4 爬升到飛控設定的起飛高度。觀察狀態
   直到 AIRBORNE / AUTO.LOITER（即 PX4 Hold），再輸入 offboard。
2. 依序輸入 x、y、z。單位為公尺，Enter 表示 0，三項填完才會送出。
   +x 為本地北向，+y 為本地東向，+z 為向上，負值往反方向。
   每筆是相對位移。首次在穩定懸停時記下位置，之後從已到達並保持的上一筆
   終點出發。例：x=1、y=1、z=0，沿同一條對角線移動約 1.414 m。
   當地 NED 座標對準地理北時，heading 0° 才是地理北。
3. 初次進入前以 50 Hz 預送懸停目標至少 1.2 秒，確認 PX4 已進 Offboard。
   機頭沿最短方向逐步轉正，目標角速度上限 15°/s。指令到 0°、最新實際
   heading 在 ±5° 內後立即開始平移，沒有固定倒數。
4. 各軸共用半餘弦進度。若直線長 D，移動時間 T = πD / (2 × 0.1) 秒，
   目標的三維合成速度最高 0.1 m/s。此上限作用於 Offboard 平移參考；
   監控端另顯示實際速度。自動起飛與降落使用 PX4 自身的速度設定。
5. 到達時須有新鮮遙測：距終點 ≤ 0.05 m、實測速度 ≤ 0.05 m/s、
   heading 在 ±5° 內，持續至少 0.5 秒。畫面顯示「指令完成，等待下個指令」。
   控制節點繼續送終點與 yaw=0；監控端停刷例行狀態文字，持續更新曲線與事件。
   輸入下一筆 offboard 即可銜接。執行中收到的新位移會回報忙碌。
6. 輸入 land 可中斷移動並請 PX4 就地降落。在 x/y/z 提示中也可直接輸入
   land；cancel 只取消未送出的輸入。等待地面＋上鎖回報才顯示收尾完成。
   鍵盤 quit 僅關閉鍵盤。關閉 main.py 會中止控制串流，不等同下達 land。

PX4 與介面
-----------
無手動輸入時可設 COM_RC_IN_MODE=4。此流程使用起飛後進入 Hold 的設定
COM_TAKEOFF_ACT=0。飛控依自身解鎖條件及機上安全開關判斷是否放行。
應可取得有效的 PX4 估測及 Hold 定位，並核對實機的 COM_OF_LOSS_T、
COM_OBL_RC_ACT。停止目標串流後，由 PX4 依自身失聯設定處理。
參考 https://docs.px4.io/v1.17/en/flight_modes/offboard

/flight/request  std_msgs/String，單筆 JSON，採用新 id 以區別不同請求。
    {"id":"001","mode":"takeoff"}
    {"id":"002","mode":"offboard","x":1.0,"y":1.0,"z":0.0}
    {"id":"003","mode":"land"}
/flight/state    std_msgs/String，10 Hz JSON；包含 PX4 模式、armed、著陸狀態、
                 heading、三維實際速度大小、實測與指令 NED 座標及程式階段。
/flight/event    std_msgs/String，命令與飛控事件。ACK 後仍須等遙測確認。
監控以同一個本地原點繪製 North、East、Up。命令曲線顯示本程式送出的
Offboard 目標，自動起降期間僅顯示已知的實測位置。

離線測試（標準 Python 3.10+）
    python3 -B test_flight.py -v
