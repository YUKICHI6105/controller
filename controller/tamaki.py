import rclpy
import threading
import asyncio
import server

def main():
  # stop_source = いいかんじのクラスを作ろう/ライブラリを漁ろう
  # channel_hoge = ROS/server間のデータの並行なやりとりのためのオブジェクトを用意しよう
  # channel_fuga = ...
  # channel_piyo = ...

  def ros_thread_func(stop_flag) -> None:
    # rclpyの初期化処理
    # rclpy.init ...
    
    # ノードの作成
    # node = ...
    while not stop_flag.stop_requested():
      rclpy.spin_once(node)

  ros_thread = threading.Thread(target=ros_thread_func, args=(controller, stop_source.generate_flag()))
 
  # asyncioなんもわからんので、何して良くて何がダメなんかわからんが...
  async def server_run(stop_flag):
    # serverの初期化処理
    # server = うんたらかんたら
    while not stop_flag.stop_requested():
      await server.run_once()  # serverの処理を細切れにして何度も呼ぶ、という方針にする
    # serverの終了処理
    # ...

  def server_thread_func(stop_flag) -> None:
    asyncio.run(server_run(stop_flag))
  server_thread = threading.Thread(target=server_thread_func, args=(stop_source.generate_flag(),))

  # デストラクタが無くて可哀想
  ros_thread.join()
  server_thread.join()