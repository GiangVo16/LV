const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Motor Control</title>
  <style>
    body {
      display: flex;
      flex-direction: column;
      align-items: center;
      font-family: Arial, sans-serif;
    }
    .slider-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      margin: 20px 0;
    }
    .slider {
      width: 300px;
      margin-top: 10px;
    }
    .button-grid {
      display: grid;
      grid-template-columns: repeat(3, auto);
      gap: 10px;
      margin-top: 20px;
    }
    .button {
      padding: 10px 20px;
      font-size: 16px;
      border: 2px solid black;
      border-radius: 5px;
      cursor: pointer;
    }
    .empty-cell {
      visibility: hidden;
    }
    .status {
      margin-top: 20px;
      font-size: 18px;
    }
  </style>
</head>
<body>
  <h2>Speed</h2>
  <span id="sliderValue">127</span> <!-- Giá trị slider mặc định là 127 -->
  
  <div class="slider-container">
    <input type="range" min="0" max="255" class="slider" id="speed" oninput="updateSpeed(this.value)" style="background-color: blue;">
  </div>

  <!-- Hiển thị giá trị của biến variable_esp32 -->
  <h3>Giá trị biến từ ESP32: <span id="variableValue">--</span></h3> 

  <!-- Hiển thị tọa độ (x, y, θ) -->
  <div class="status">
    <h3>Tọa độ robot</h3>
    <p>X: <span id="xCoord">--</span></p>
    <p>Y: <span id="yCoord">--</span></p>
    <p>θ: <span id="theta">--</span></p>
  </div>

  <div class="button-grid">
    <button class="button empty-cell"></button>
    <button class="button" onmousedown="sendData('forward')" onmouseup="sendData('stop')" 
            ontouchstart="sendData('forward')" ontouchend="sendData('stop')">Tiến</button>
    <button class="button empty-cell"></button>
    
    <button class="button" onmousedown="sendData('left')" onmouseup="sendData('stop')" 
            ontouchstart="sendData('left')" ontouchend="sendData('stop')">Trái</button>
    <button class="button empty-cell"></button>
    <button class="button" onmousedown="sendData('right')" onmouseup="sendData('stop')" 
            ontouchstart="sendData('right')" ontouchend="sendData('stop')">Phải</button>
    
    <button class="button empty-cell"></button>
    <button class="button" onmousedown="sendData('backward')" onmouseup="sendData('stop')" 
            ontouchstart="sendData('backward')" ontouchend="sendData('stop')">Lùi</button>
    <button class="button empty-cell"></button>
  </div>

  <script>
    const ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => console.log('WebSocket Connected');
    ws.onclose = () => console.log('WebSocket Disconnected');

    ws.onmessage = (event) => {
      // Xử lý dữ liệu nhận được từ ESP32
      const data = event.data;

      if (data.startsWith("variable_esp32:")) {
        document.getElementById("variableValue").textContent = data.split(":")[1];
      } else if (data.startsWith("pose:")) {
        // Dữ liệu tọa độ có dạng: pose:x,y,theta
        const [x, y, theta] = data.split(":")[1].split(",");
        document.getElementById("xCoord").textContent = x;
        document.getElementById("yCoord").textContent = y;
        document.getElementById("theta").textContent = theta;
      }
    };

    function sendData(message) {
      ws.send(message);
    }

    function updateSpeed(value) {
      document.getElementById("sliderValue").textContent = value;
      ws.send("speed" + value);
    }
  </script>
</body>
</html>
)rawliteral";
