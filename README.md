# Controller
このpackeageはwifiを利用して手元のスマホからコントローラーの入力をROSに送信するためのものです。おもに大規模なロボコン(NHK学生ロボコン)などを想定しています。

また、暗号化通信が必須なweaklockやsleep blockなどを使用しており、今後のブラウザーの既定によっては自己証明書だけでは正しく動作しなくなる可能性があります。

## Usage
はじめに自己証明を導入するためopensslで鍵を生成し、利用する端末に公開鍵を入れます。  
(Windows11でOpenSSLを利用するための方法)[https://qiita.com/SKY-HaYaTo/items/7c40256543a4f9d14c0d]

certificate.pem  
key.pem

この2つのうちどちらかが公開鍵でどちらかが秘密鍵です。  
これをcontrollerのソースがあるcontroller/controllerディレクトリに入れてください。  
端末に公開鍵を導入するのを忘れないでください  
同時にserver.pyにある鍵のパスを編集してください。

ファイヤーウォールを解除必須
