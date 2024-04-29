import React, {Children, useCallback, useRef, useEffect, useState, PureComponent, useContext} from 'react';
import ReactDOM from 'react-dom/client';
import PortalReactDOM from 'react-dom'
import util from 'util'
import './index.css';
import App from './App';
import reportWebVitals from './reportWebVitals';
import DatalistInput from 'react-datalist-input';
import 'react-datalist-input/dist/styles.css';
import { Async, createInstance, useAsync } from 'react-async';
import { useHooksInCallback } from "react-hooks-in-callback";
import { invoke } from '@tauri-apps/api';
import { Serialport } from 'tauri-plugin-serialport-api';
import ZoomPlugin from 'chartjs-plugin-zoom';
import moment from "moment";
import { Canvas, useFrame, useLoader } from '@react-three/fiber'
import { useGLTF, Stage, Grid, OrbitControls, Environment, Html } from '@react-three/drei'
import { emit, listen } from '@tauri-apps/api/event'
import {BaseDirectory, createDir, readDir, readTextFile, removeFile, writeFile} from '@tauri-apps/api/fs'
import {appDataDir} from '@tauri-apps/api/path';
import { Dygraph } from '@qogni/react-dygraphs';
import AHRS from 'ahrs';

//import { useGLTF, Stage, Grid, OrbitControls, Environment } from '@react-three/drei'
//import { EffectComposer, Bloom } from '@react-three/postprocessing'
//import { easing } from 'maath'
//import { MapContainer, Marker, Popup, TileLayer } from 'react-leaflet';
//import 'leaflet/dist/leaflet.css';
import { YMaps, Map, useYMaps, Placemark, Polyline } from '@pbe/react-yandex-maps';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
} from 'chart.js';
import { Line } from 'react-chartjs-2';
import { StreamingPlugin, RealTimeScale } from "chartjs-plugin-streaming";
import 'chartjs-adapter-moment';
import { touchProps } from 'react-native-web/dist/cjs/modules/forwardedProps';
const pluginBlack = {
  id: 'customCanvasBackgroundColor',
  beforeDraw: (chart, args, options) => {
    const {ctx} = chart;
    ctx.save();
    ctx.globalCompositeOperation = 'destination-over';
    ctx.fillStyle = options.color || '#99ffff';
    ctx.fillRect(0, 0, chart.width, chart.height);
    ctx.restore();
  }
};
const madgwick = new AHRS({
  /*
   * The sample interval, in Hz.
   *
   * Default: 20
   */
  sampleInterval: 1,

  /*
   * Choose from the `Madgwick` or `Mahony` filter.
   *
   * Default: 'Madgwick'
   */
  algorithm: 'Madgwick',

  /*
   * The filter noise value, smaller values have
   * smoother estimates, but have higher latency.
   * This only works for the `Madgwick` filter.
   *
   * Default: 0.4
   */
  beta: 30.,

  /*
   * The filter noise values for the `Mahony` filter.
   */
  kp: 0.5, // Default: 0.5
  ki: 0, // Default: 0.0

  /*
   * When the AHRS algorithm runs for the first time and this value is
   * set to true, then initialisation is done.
   *
   * Default: false
   */
  doInitialisation: false,
});
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  RealTimeScale,
  Title,
  Tooltip,
  Legend,
  StreamingPlugin,
  ZoomPlugin,
  pluginBlack,
);
appDataDir().then(q => {
  readDir(q).then(d => {
    console.log(d)
  })
})
var coordination = readTextFile('./smthh.txt', {dir: BaseDirectory.App});
var arr = [];
removeFile("./sometext.txt", {dir: BaseDirectory.App})

coordination.then(
  coordinates => {

  },
  error => {
    // вторая функция - запустится при вызове reject
    createDir("data", {dir: BaseDirectory.App, recursive: true,});
    writeFile({contents: "[DATATATATA]", path: './smthh.txt'}, {dir: BaseDirectory.App});
  }
);
var coordinationt = readTextFile('./smth.txt', {dir: BaseDirectory.App});
coordinationt.then(
  coordinates => {

  },
  error => {
    // вторая функция - запустится при вызове reject
    writeFile({contents: "[DATATATATA]", path: './smth.txt'}, {dir: BaseDirectory.App});
  }
);
var coordination = readTextFile('./sometext.txt', {dir: BaseDirectory.App});
var arr = [];
coordination.then(
  coordinates => {

  },
  error => {
    // вторая функция - запустится при вызове reject
    writeFile({contents: "", path: './sometext.txt'}, {dir: BaseDirectory.App});
  }
);

var serialport;
var globaldata = []
var PicoData = [[0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]
var mainstatus = "Not opened"
const UserContext = React.createContext(null);
serialport = new Serialport({path:'COM10', baudRate: 115200});
var event = new CustomEvent("name-of-event", { "detail": "Example of an event" });
serialport.close()
/*
serialport.open().then((res) =>{
  console.log(res)
})
.catch((err) => {
  console.log(err)
})
*/
const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
); 
class SK {
  constructor(err_measure, oprosspeed) {
    this._err_measure = parseFloat(err_measure);
    this._q = parseFloat(oprosspeed);
    this._kalman_gain = 0.;
    this._current_estimate = 0.;
    this._last_estimate = 0.
    this._err_estimate = parseFloat(err_measure);
  }

  calc(newval) {
    this._kalman_gain = this._err_estimate / (this._err_estimate + this._err_measure);
    this._current_estimate = this._last_estimate + this._kalman_gain * (newval - this._last_estimate);
    this._err_estimate = (1. - this._kalman_gain) * this._err_estimate + Math.abs(this._last_estimate - this._current_estimate) * this._q;
    this._last_estimate = this._current_estimate;
    return parseFloat(this._current_estimate);
  }
}
var smax = new SK(0.15, 0.05);
var smay = new SK(0.15, 0.05);
var smaz = new SK(0.15, 0.05);
var smmx = new SK(20., 0.05);
var smmy = new SK(20., 0.05);
var smmz = new SK(20., 0.05);
var smgx = new SK(0.15, 0.05);
var smgy = new SK(0.15, 0.05);
var smgz = new SK(0.15, 0.05);
var date = new Date();

var itemso = []
var terminalData = []
var Accelerometer = [0.,0.,0.]
var Gyroscope = [0.,0.,0.]
var Magnetometer = [0., 0., 0.]
var Temp0 = 0 
var Temp1 = 0
var Temp2 = 0
var Temp3 = 0
var Temp4 = 0
var Baromether = 0
var Latitude = ""
var Longitude = ""
var Latitude1 = ""
var Longitude1 = ""
var angles = [0., 0., 0.]
var Timez = 0
var processingcounter = 0
function PressToHeight(p){
	return 44330 * (1.0 - Math.pow(p/101325, 0.1903));
}
function appendToTerminal(str){
  terminalData = []
  terminalData.push(str+`<br>`)
  /*
  if(terminalData.length > 10){
    terminalData.shift()
  }
  */
  var inner = document.getElementById("SerialResults")
  inner.innerHTML = terminalData
}
const inner = document.getElementById("serialResults");
const listener = async() => {
  while (true){
    await new Promise(resolve => setTimeout(resolve, 1000));
    
    serialport.read({timeout: 999})
    .then((res) => {
      console.log(Accelerometer, Gyroscope, Magnetometer);   
      var inner = document.getElementById("SerialResults")
  
      document.dispatchEvent(event);
      document.body.dispatchEvent(event);
      console.log(Timez)
    })
    .catch((err) => {
      console.error(err)
    })
    await listen('plugin-serialport-read', (event) =>{
      //console.log(event.payload)
      var strin = ""
      for (var i = 0; i < event.payload.size; i++){
        strin += String.fromCharCode(event.payload.data[i])
      }
      var carrr = strin.split("\n");
      for (var i = 0; i < carrr.length; i++){
        var b = carrr[i].split(";")
        if (b[0] == "MAIN"){
          if (b[1] == "0") mainstatus = "Sputnikovaya Set' has started"
          Accelerometer = [parseFloat(b[2]), parseFloat(b[3]), parseFloat(b[4])]
          Gyroscope = [parseFloat(b[5]), parseFloat(b[6]), parseFloat(b[7])]
          Magnetometer = [parseFloat(b[8]), parseFloat(b[9]), parseFloat(b[10])]
          Temp0 = parseFloat(b[11])
          Baromether = parseInt(b[12])
          Temp1 = parseFloat(b[13])
          Temp2 = parseFloat(b[14])
          Temp3 = parseFloat(b[15])
          Temp4 = parseFloat(b[16])
          Latitude1 = b[17]
          Longitude1 = b[18]
          Latitude = String(parseFloat(b[17].slice(0, 2))
            +parseFloat(
              b[17].slice(2, b[17].length).slice(0, 2) + "." + b[17].slice(2, b[17].length).slice(2, b[17].slice(2, b[17].length).length)  
            )/60.)
            if (b[18][0] != "1") b[18] = '0'+b[18]
          Longitude = String(parseFloat(b[18].slice(0, 3))
            +parseFloat(
              b[18].slice(3, b[18].length).slice(0, 2) + "." + b[18].slice(3, b[18].length).slice(2, b[18].slice(3, b[18].length).length) 
            )/60.)
          
        }
        if (b[0] == "PICO1"){
          PicoData[0][0] = parseFloat(b[1]) // Ax
          PicoData[0][1] = parseFloat(b[2]) // Ay
          PicoData[0][2] = parseFloat(b[3]) // Az
        
          PicoData[0][3] = parseFloat(b[4]) // Gx
          PicoData[0][4] = parseFloat(b[5]) // Gy
          PicoData[0][5] = parseFloat(b[6]) // Gz

          PicoData[0][6] = parseFloat(b[7]) // Mx
          PicoData[0][7] = parseFloat(b[8]) // My
          PicoData[0][8] = parseFloat(b[9]) // Mz

          PicoData[0][9] = parseFloat(b[10]) // Temp
          PicoData[0][10] = parseFloat(b[11]) // Bar

          PicoData[0][11] = b[12] // Lat
          PicoData[0][12] = b[13] // Lon
          PicoData[0][13] = parseFloat(b[14]) // Height
        }
        if (b[0] == "PICO2"){
          PicoData[1][0] = parseFloat(b[1]) // Ax
          PicoData[1][1] = parseFloat(b[2]) // Ay
          PicoData[1][2] = parseFloat(b[3]) // Az
        
          PicoData[1][3] = parseFloat(b[4]) // Gx
          PicoData[1][4] = parseFloat(b[5]) // Gy
          PicoData[1][5] = parseFloat(b[6]) // Gz

          PicoData[1][6] = parseFloat(b[7]) // Mx
          PicoData[1][7] = parseFloat(b[8]) // My
          PicoData[1][8] = parseFloat(b[9]) // Mz

          PicoData[1][9] = parseFloat(b[10]) // Temp
          PicoData[1][10] = parseFloat(b[11]) // Bar

          PicoData[1][11] = b[12] // Lat
          PicoData[1][12] = b[13] // Lon
          PicoData[1][13] = parseFloat(b[14]) // Height
        }
        if (b[0] == "PICO3"){
          PicoData[2][0] = parseFloat(b[1]) // Ax
          PicoData[2][1] = parseFloat(b[2]) // Ay
          PicoData[2][2] = parseFloat(b[3]) // Az
        
          PicoData[2][3] = parseFloat(b[4]) // Gx
          PicoData[2][4] = parseFloat(b[5]) // Gy
          PicoData[2][5] = parseFloat(b[6]) // Gz

          PicoData[2][6] = parseFloat(b[7]) // Mx
          PicoData[2][7] = parseFloat(b[8]) // My
          PicoData[2][8] = parseFloat(b[9]) // Mz

          PicoData[2][9] = parseFloat(b[10]) // Temp
          PicoData[2][10] = parseFloat(b[11]) // Bar

          PicoData[2][11] = b[12] // Lat
          PicoData[2][12] = b[13] // Lon
          PicoData[2][13] = parseFloat(b[14]) // Height
        }
        if (b[0] == "PICO4"){
          PicoData[3][0] = parseFloat(b[1]) // Ax
          PicoData[3][1] = parseFloat(b[2]) // Ay
          PicoData[3][2] = parseFloat(b[3]) // Az
        
          PicoData[3][3] = parseFloat(b[4]) // Gx
          PicoData[3][4] = parseFloat(b[5]) // Gy
          PicoData[3][5] = parseFloat(b[6]) // Gz

          PicoData[3][6] = parseFloat(b[7]) // Mx
          PicoData[3][7] = parseFloat(b[8]) // My
          PicoData[3][8] = parseFloat(b[9]) // Mz

          PicoData[3][9] = parseFloat(b[10]) // Temp
          PicoData[3][10] = parseFloat(b[11]) // Bar

          PicoData[3][11] = b[12] // Lat
          PicoData[3][12] = b[13] // Lon
          PicoData[3][13] = parseFloat(b[14]) // Height
        }

      }
      console.log(strin)
      /*
      if (Gyroscope[0] < 5){
        Gyroscope[0] = 0
      }
      if (Gyroscope[1] < 5){
        Gyroscope[1] = 0
      }
      if (Gyroscope[2] < 5){
        Gyroscope[2] = 0
      }
      */
      /*
      Accelerometer[0] = smax.calc(Accelerometer[0])
      Accelerometer[1] = smay.calc(Accelerometer[1])
      Accelerometer[2] = smaz.calc(Accelerometer[2])
      Gyroscope[0] = smgx.calc(Gyroscope[0])
      Gyroscope[1] = smgy.calc(Gyroscope[1])
      Gyroscope[2] = smgz.calc(Gyroscope[2])
      Magnetometer[0] = smmx.calc(Magnetometer[0])
      Magnetometer[1] = smmy.calc(Magnetometer[1])
      Magnetometer[2] = smmz.calc(Magnetometer[2])
      */

      //appendToTerminal(strin)
    })
    /*
    madgwick.update(Gyroscope[0] * (Math.PI / 180), Gyroscope[1] * (Math.PI / 180), Gyroscope[2] * (Math.PI / 180), Accelerometer[0] / 9.81, Accelerometer[1] / 9.81, Accelerometer[2] / 9.81, Magnetometer[0], Magnetometer[1], Magnetometer[2]);
      var temp = madgwick.getEulerAngles();
      angles[0] = temp?.pitch;
      angles[1] = temp?.heading;
      angles[2] = temp?.roll;
    */
    //document.dispatchEvent(event);
    //document.body.dispatchEvent(event);

  }
}
//var event = new CustomEvent("name-of-event", {"detail": 115200})
var opened = 0;
export const Serial = () => {
  const [item, setItem] = useState();
  const [num, setNum] = useState(0);
  const [baud, setBaud] = useState(0);
  const [placehold, setPlacehold] = useState("Select baudrate")
  //const onSelect = useCallback((selectedItem) => {
    
    //inner.innerHTML = ''
    //inner.innerHTML += `> ${selectedItem.value}<br>`;
    
  //})
  useEffect(()=>{
  })
  async function OnSelect(selectedItem){
    if(num == 0){
      //console.log('selectedItem', selectedItem.value);
      const inner = document.getElementById("SerialResults")
      setItem(selectedItem.value)
      Serialport.available_ports().then((res) => {
        //console.log('available_ports: ', res);
        for (var i = 0; i < res.length; i++){
          itemso.push({id: res[i], value: res[i]});
        }
        setNum(1)
        setBaud(selectedItem.value)
      })
      .catch((err) => {
        console.error(err);
      });
      
      
    } else {
      
      serialport = new Serialport({path: selectedItem.value, baudRate: parseInt(item)});
      if(!opened) serialport.open().then((res)=>{
        console.log("Yuppy")
        opened = 1
        Timez = 0
        setNum(0)
      })
      .catch((err) => {
        console.log("Mehh")
        itemso = []
        setNum(0)
      })
      setPlacehold(`${selectedItem.value}`)
    }
    
  }
  if (num == 0)
    return(
    <div>
      <Async promiseFn={listener}>
      </Async>
      <button onClick={() => {
        console.log(document
          .getElementById("textboxi").value);
        serialport.open().then((res) =>{
          console.log(`sending value on ${res}<br>`)
          serialport.write(document.getElementById("textboxi").value)
        }).catch((err) => {
          console.log("No serials?")
        })
      }}>Send</button>
      <input id = "textboxi" type="text" name="located" list="items"/>
    <div id = "DataList">
    
    <DatalistInput
    placeholder={`${placehold}`}
    label="Connect"
    onSelect={OnSelect}
    items={[
      { id: '9600', value: '9600' },
      { id: '19200', value: '19200' },
      { id: '38400', value: '38400' },
      { id: '57600', value: '57600' },
      { id: '115200', value: '115200' },
    ]}
    />
    </div>
    </div>
  )
  if (num == 1)
    return(
    //<button onClick={()=>{console.log("negro")}}>Send</button>
    <div>
      <button onClick={() => {
        console.log(document
          .getElementById("textboxi").value);
          
      }}>Send</button>
      <input id = "textboxi" type="text" name="located" list="items"/>
    <div id = "DataList">
    <DatalistInput
    placeholder={`${baud}`}
    label="Connect"
    onSelect={OnSelect}
    items={itemso}
  
    />
    </div>
    </div>
    )
};
export const TextBox = () => {
  return(
    <div id = "SerialResults"><h4></h4></div>
  )
};
const chartColors = {
  red: "rgb(255, 99, 132)",
  orange: "rgb(255, 159, 64)",
  yellow: "rgb(255, 205, 86)",
  green: "rgb(75, 192, 192)",
  blue: "rgb(54, 162, 235)",
  purple: "rgb(153, 102, 255)",
  grey: "rgb(201, 203, 207)"
};
var fullPresets = {
  axes: {
    x: {
      drawGrid: false,
      drawAxis: true,
      axisLineColor: "white",
      axisLineWidth: 1.5
    },
    y: {
      drawAxis: true,
      gridLineWidth: 1.5,
      gridLineColor: "#eee",
      gridLinePattern: [5, 5],
      axisLineColor: "white",
      axisLineWidth: 1
    }
  },
  rollPeriod: 10,
  highlightCircleSize: 5,
  labels: ["X", "Y1", "Y2"],
  legend: "follow",
  strokeWidth: 2,
  fillGraph: true,
  colors: ["#f47560", "#61cdbb"],
  visibility: [true, true],
  animatedZooms: true,
  hideOverlayOnMouseOut: false
}
class PlotReactive extends React.Component{
  constructor(props){
    super(props)
    this.ref = React.createRef(null)
    this.state = {
      data: [1, 1]
    }
    this.buffer = [1, 1]
  }//DyGraphs: Интегрировать графики в класс, присвоить их state'у, привязать ивент листенер и через него обновлять стейт графиков
  _handleNVEvent = event => {
    this.buffer = this.state.data
    this.buffer.push(moment().toString(), Math.random().toString())
    this.setState({
      data: this.buffer
    })
  } 

  componentDidMount() {
    new Dygraph(this.ref.current, this.state.data, fullPresets);
    document.body.addEventListener('name-of-event', this._handleNVEvent.bind(this));
  }
  componentWillUnmount() {
    document.body.removeEventListener('name-of-event', this._handleNVEvent.bind(this));
  }
  render (){
    return(
      <div>
        <div ref={this.ref} />
      </div>
    )
  }
}
class DyGraph extends React.Component{
  constructor(props){
    super(props)
    this.counter = 0
    this.state = {
      data: [[this.counter, 0]]
    }
  }
  _handleNVEvent = event => {
    this.counter += 1
    if (this.props.name == "bar" && Baromether != 0){
      this.setState({
        data: this.state.data.concat([[Timez, Baromether]])
      })
    }
    if (this.props.name == "temp" && Temp1 != 0){
      this.setState({
        data: this.state.data.concat([[Timez, Temp1]])
      })
    }
    if (this.props.name == "temp2" && Temp2 != 0){
      this.setState({
        data: this.state.data.concat([[Timez, Temp2]])
      })
    }
    if (this.props.name == "accel" && Math.sqrt(Math.pow(Accelerometer[0], 2)+Math.pow(Accelerometer[1], 2)+Math.pow(Accelerometer[2], 2)) != 0){
      this.setState({
        data: this.state.data.concat([[Timez, Math.sqrt(Math.pow(Accelerometer[0], 2)+Math.pow(Accelerometer[1], 2)+Math.pow(Accelerometer[2], 2))]])
      })
    }
    
    var somestringu = "Time: " + String(Timez) + "\n" + "Pressure: " + String(Baromether) + "\n" + "Accelerometer " + 
    String(Math.sqrt(Math.pow(Accelerometer[0], 2)+Math.pow(Accelerometer[1], 2)+Math.pow(Accelerometer[2], 2))) + "<br>" + "Temperature " + Temp1 + "<br>" +
    "Lat "+String(Latitude) + " Lon "+String(Longitude) + '<br>' +
    "P1 Lat "+PicoData[0][11] + " Lon "+PicoData[0][12] + '<br>' +
    "P2 Lat "+PicoData[1][11] + " Lon "+PicoData[1][12] + '<br>' +
    "P3 Lat "+PicoData[2][11] + " Lon "+PicoData[2][12] + '<br>' +
    "P4 Lat "+PicoData[3][11] + " Lon "+PicoData[3][12] + '<br>' 
    appendToTerminal(somestringu)
    if(parseFloat(Latitude) && parseFloat(Longitude) 
       && parseFloat(PicoData[0][11]) && parseFloat(PicoData[0][12]) 
       && parseFloat(PicoData[1][11]) && parseFloat(PicoData[1][12])
       && parseFloat(PicoData[2][11]) && parseFloat(PicoData[2][12])
       && parseFloat(PicoData[3][11]) && parseFloat(PicoData[3][12])
      )
    {
      var tempcoord = [
        [parseFloat(Latitude)*Math.PI/180, parseFloat(Longitude)*Math.PI/180],
        [parseFloat(PicoData[0][11])*Math.PI/180, parseFloat(PicoData[0][12])*Math.PI/180],
        [parseFloat(PicoData[1][11])*Math.PI/180, parseFloat(PicoData[1][12])*Math.PI/180],
        [parseFloat(PicoData[2][11])*Math.PI/180, parseFloat(PicoData[2][12])*Math.PI/180],
        [parseFloat(PicoData[3][11])*Math.PI/180, parseFloat(PicoData[3][12])*Math.PI/180]
      ]
      var resultarr = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
      var rad = 6372795
      for(var i = 1; i < 5; i++){
        var cl1 = Math.cos(tempcoord[0][0])
        var cl2 = Math.cos(tempcoord[i][0])
        var sl1 = Math.sin(tempcoord[0][0])
        var sl2 = Math.sin(tempcoord[i][0])
        var delta = tempcoord[i][1] - tempcoord[0][1]
        var cdelta = Math.cos(delta)
        var sdelta = Math.sin(delta)
        var y = Math.sqrt(Math.pow(cl2*sdelta,2)+Math.pow(cl1*sl2-sl1*cl2*cdelta,2))
        var x = sl1*sl2+cl1*cl2*cdelta
        var ad = Math.atan2(y,x)
        var dist = ad*rad
        x = (cl1*sl2) - (sl1*cl2*cdelta)
        y = sdelta*cl2
        var z = Math.atan(-y/x)*180/Math.PI

        if (x < 0)
        z = z+180.

        var z2 = (z+180.) % 360. - 180.
        z2 = -(z2)*Math.PI/180
        var anglerad2 = z2 - ((2*Math.PI)*Math.floor((z2/(2*Math.PI))) )
        var angledeg = (anglerad2*180.)/Math.PI

        resultarr[i][0] = Math.sin(anglerad2)*(dist/1000)
        resultarr[i][1] = Math.cos(anglerad2)*(dist/1000)
        resultarr[i][2] = PressToHeight(Baromether) - PressToHeight(PicoData[i-1][10])
      }
      console.log(resultarr)
      if(Timez > 10)invoke('update_processing', {jsMsg: `\n${resultarr[0][0]};${resultarr[0][1]};${resultarr[0][2]};${Temp1};${resultarr[1][0]};${resultarr[1][1]};${resultarr[1][2]};${PicoData[0][9]};${resultarr[2][0]};${resultarr[2][1]};${resultarr[2][2]};${PicoData[1][9]};${resultarr[3][0]};${resultarr[3][1]};${resultarr[3][2]};${PicoData[2][9]};${resultarr[4][0]};${resultarr[4][1]};${resultarr[4][2]};${PicoData[3][9]}`})
      if(Timez > 10)invoke('update_coord', {jsMsg: `\n${resultarr[0][0]};${resultarr[0][1]};${resultarr[0][2]};${Temp1};${resultarr[1][0]};${resultarr[1][1]};${resultarr[1][2]};${PicoData[0][9]};${resultarr[2][0]};${resultarr[2][1]};${resultarr[2][2]};${PicoData[1][9]};${resultarr[3][0]};${resultarr[3][1]};${resultarr[3][2]};${PicoData[2][9]};${resultarr[4][0]};${resultarr[4][1]};${resultarr[4][2]};${PicoData[3][9]}`})
    }
    var currentdate = new Date(); 
    var datetime = currentdate.getDate() + "/" + (currentdate.getMonth()+1) + "/" + currentdate.getFullYear()+":"+ currentdate.getHours() + ":"  + currentdate.getMinutes() + ":" + currentdate.getSeconds();
    invoke('update_txt', {jsMsg: `Main;${Timez};${datetime};${Baromether};${Accelerometer[0]};${Accelerometer[1]};${Accelerometer[2]};${Gyroscope[0]};${Gyroscope[1]};${Gyroscope[2]};${Magnetometer[0]};${Magnetometer[1]};${Magnetometer[2]};${Temp0};${Temp1};${Temp2};${Temp3};${Temp4};${Latitude1};${Longitude1};${Latitude};${Longitude};\nPico1;${PicoData[0][0]};${PicoData[0][1]};${PicoData[0][2]};${PicoData[0][3]};${PicoData[0][4]};${PicoData[0][5]};${PicoData[0][6]};${PicoData[0][7]};${PicoData[0][8]};${PicoData[0][9]};${PicoData[0][10]};${PicoData[0][11]};${PicoData[0][12]};${PicoData[0][13]};\nPico2;${PicoData[1][0]};${PicoData[1][1]};${PicoData[1][2]};${PicoData[1][3]};${PicoData[1][4]};${PicoData[1][5]};${PicoData[1][6]};${PicoData[1][7]};${PicoData[1][8]};${PicoData[1][9]};${PicoData[1][10]};${PicoData[1][11]};${PicoData[1][12]};${PicoData[1][13]};\nPico3;${PicoData[2][0]};${PicoData[2][1]};${PicoData[2][2]};${PicoData[2][3]};${PicoData[2][4]};${PicoData[2][5]};${PicoData[2][6]};${PicoData[2][7]};${PicoData[2][8]};${PicoData[2][9]};${PicoData[2][10]};${PicoData[2][11]};${PicoData[2][12]};${PicoData[2][13]};\nPico4;${PicoData[3][0]};${PicoData[3][1]};${PicoData[3][2]};${PicoData[3][3]};${PicoData[3][4]};${PicoData[3][5]};${PicoData[3][6]};${PicoData[3][7]};${PicoData[3][8]};${PicoData[3][9]};${PicoData[3][10]};${PicoData[3][11]};${PicoData[3][12]};${PicoData[3][13]};\n`});
  } 

  componentDidMount() {
    document.body.addEventListener('name-of-event', this._handleNVEvent.bind(this));
  }
  componentWillUnmount() {
    document.body.removeEventListener('name-of-event', this._handleNVEvent.bind(this));
  }
  render(){
    return(
      <div>
        <Dygraph data = {this.state.data}>

        </Dygraph>
      </div>
    )
  }
}

class Plot1 extends React.Component{
  constructor(props){
    super(props);
    this.data = {
      datasets: [
        {
          label: "Dataset 1 (linear interpolation)",
          backgroundColor: 0,
          borderColor: chartColors.red,
          fill: false,
          lineTension: 0.05,
          borderDash: [0],
          data: [],
          pointRadius: 2
        }
      ]
    };
    
    this.options = {
      animation: false,
      spanGaps: true,
      responsive: true,
      aspectRatio: 1.54 ,
      plugins: {
        customCanvasBackgroundColor: {
          color: 'black',
        },
        legend: {
          position: 'top',
        },
        title: {
          display: true,
          text: 'Chart.js Line Chart',
        },
        zoom: {
          pan: {
            enabled: true,
            mode: 'x'
          },
          zoom: {
            pinch: {
              enabled: true
            },
            wheel: {
              enabled: true
            },
            mode: 'x'
          },
          limits: {
            x: {
              minDelay: 0,
              maxDelay: 5000000,
              minDuration: 10000,
              maxDuration: 5000000,
            }
          }
        }
      },
      scales: {
        x:{
          beginAtZero: true,
          type: 'realtime',
          distribution: "linear",
          realtime: {
            ttl: 10000000,
            duration: 5000000,
            delay: 500,
            time: {
              displayFormat: "mm:ss"
            },
          },
          ticks: {
            displayFormats: 2,
            maxRotation: 0,
            minRotation: 0,
            samplesize: 2,
            stepSize: 1,
            maxTicksLimit: 100,
            minUnit: "second",
            source: "auto",
            autoSkip: true,
            
          },
        },
        y:{
          ticks:{
            maxTicksLimit: 100,
            samplesize: 2
          }
        }
      },
    };
  }
  _handleNVEvent = event => {
    //("Negretyonok...")
    this.data.datasets[0].data.push({
      x: moment(),
      y: Math.random()
    })
  } 
  componentDidMount() {
    document.body.addEventListener('name-of-event', this._handleNVEvent.bind(this));
  }
  componentWillUnmount() {
    document.body.removeEventListener('name-of-event', this._handleNVEvent.bind(this));
  }
  render(){
    return(
      <div>
        <Line data = {this.data} options = {this.options}/>
      </div>
    )
  }
}
/*
class Osm extends React.Component{
  constructor(props){
    super(props)
    this.position = [51.505, -0.09]
  }
  render(){
    return(
      <MapContainer center={this.position} zoom={13} scrollWheelZoom={false}>
    <TileLayer
      attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
    />
    <Marker position={this.position}>
      <Popup>
        A pretty CSS3 popup. <br /> Easily customizable.
      </Popup>
    </Marker>
    </MapContainer>
    )
  }
}
*/
/*
export default function Ymap() {
  const mapRef = useRef(null);
  const ymaps = useYMaps(['Map']);
  var portalElem = <Placemark geometry={[55.684758, 37.738521]}/>
  useEffect(() => {
    if (!ymaps || !mapRef.current) {
      return;
    }
    const ymaps_handle = event => {
      console.log(document.getElementById("misery"))
      //document.getElementById("misery").append(<Placemark geometry={[55.684758, 37.738521]}/>)
    }
    document.addEventListener("name-of-event", ymaps_handle);
    new ymaps.Map(mapRef.current, {
      center: [55.76, 37.64],
      zoom: 10
    });
    return() => {
      document.removeEventListener('name-of-event', ymaps_handle)
    }
  }, [ymaps]);
  
  return <div id = "misery" ref={mapRef} style={{ width: '320px', height: '240px' }} />;
}
*/
var counter = -1
var points = [[55.684758, 37.738521]]
class Ymap extends React.Component{
  constructor(props){
    super(props)
    this.data = []
    this.state = {
      result: [
      ]
    }
    this.el = this.state.result.slice(0).map((elem) => <Placemark geometry={elem} options={{ 
      iconColor: 'black',
      iconLayout: 'default#image',
      preset: 
      'islands#circleIcon',
      iconImageSize: [15, 22],
      iconImageOffset: [-3, -19]              
    }}/>)
    console.log(this.el)
  }
  _handleNVEvent = event => {
    this.data.push([Latitude, Longitude])
    //console.log(this.data)
    this.setState({
      result: this.data
    })
    counter += 1
    /*
    if(document.getElementById("SerialResults") != null){
      appendToTerminal("dsadasdasfasfqwfsdafsdfvewf")
    }
    */
    this.el.push(this.state.result.slice(counter).map((elem) => <Placemark geometry={elem} options={{ 
      iconColor: 'black',
      iconLayout: 'default#image',
      preset: 
      'islands#circleIcon',
      iconImageSize: [15, 22],
      iconImageOffset: [-3, -19]              
    }}/>))
  } 
  componentDidMount() {
    document.body.addEventListener('name-of-event', this._handleNVEvent.bind(this));
  }
  componentWillUnmount() {
    document.body.removeEventListener('name-of-event', this._handleNVEvent.bind(this));
  }
  render(){
    return(
      <YMaps>
        <Map id = "magenta" defaultState={{center: [55.751574, 37.573856], zoom: 5
        }} width={500} height={300}>
          {
            //this.state.result.length ? this.state.result.map(elem => <Placemark geometry={elem} options={{
            /*
            this.state.result.length ? this.state.result.slice(counter).map(elem => <Placemark geometry={elem} options={{ 
              iconColor: 'black',
              iconLayout: 'default#image',
              preset: 
              'islands#circleIcon',
              iconImageSize: [15, 22],
              iconImageOffset: [-3, -19]              
            }}/>) : null
            */
            this.state.result.length ? this.el : null
            //this.state.result.length ? <Polyline geometry = {this.state.result} options={{balloonCloseButton: false, strokeColor: "#000", strokeWidth: 4, strokeOpacity: 0.5,}}/>: null
          }
        </Map>
      </YMaps>
    )
  }
}
/*
function Model() {
  //const gltf = useGLTF('https://thinkuldeep.com/modelviewer/Astronaut.glb')
  //const gltf = useGLTF("/spootnik.glb")
  //const gltf = useGLTF('/ImageToStl.com_sborka.glb')
  const gltf = useGLTF('/spootnik2.glb')
  const mesh = useRef()
  const group = useRef()
  //console.log(mesh.current.size)
  useFrame((state, delta) => {

    
    group.current.rotation.x = angles[0]
    group.current.rotation.y = angles[1]
    group.current.rotation.z = angles[2]
    
  })
  return (
    <Stage intensity={0.5} environment="city" shadows={{ type: 'accumulative', bias: -0.001 }} adjustCamera={false}>
      <group ref={group}>
        <mesh ref = {mesh} scale = {0.01} position={[0.3, 1.2, 0] } translateZ={5}>
        <primitive object={gltf.scene} />
        </mesh>
      </group>
      <Grid renderOrder={-1} position={[0, -1.85, 0]} infiniteGrid cellSize={0.6} cellThickness={0.6} sectionSize={3.3} sectionThickness={1.5} sectionColor={[0.5, 0.5, 10]} fadeDistance={30} />
      <OrbitControls autoRotate autoRotateSpeed={0.05} enableZoom={true} makeDefault minPolarAngle={Math.PI / 2} maxPolarAngle={Math.PI / 2} />
      <Environment className = "blackie" background preset="night" blur={0.8} />
    </Stage>
  )
}
*/
        
function SubApp(){
  var [chart1, setChart1] = useState(null);
  function everyTime() {
    Timez += 1;
  }

  var myInterval = setInterval(everyTime, 1000);
  document.addEventListener("name-of-event", function(e) {
    //globaldata.push(10);
    //setChart1(globaldata)
    //console.log(chart1);
    //    <div style={{ backgroundColor: "#44014C", width: "600px", Height: "450px"}}><Plot1 id = "third" className = "blackie" parentData = {chart1}/></div>
    //    <div style={{ backgroundColor: "#44014C", width: "600px", Height: "450px"}}><Plot1 id = "fourth" className = "blackie" parentData = {chart1}/></div>
  });
  return(
    <div>
    <UserContext.Provider value={{chart1}}>
    <div className='row h-50'>
      <div id = "textboxicr">
        <div id = "textboxic"><Serial /></div>
        <div style={{ width: "499px", height: "210px"}}><TextBox /></div>
      </div>
      <div id = "first" className = "blackie col d-flex flex-column justify-content-around"><DyGraph className = "blackie" name = "bar" parentData = {chart1}/></div>
      <div id = "first" className = "blackie col d-flex flex-column justify-content-around"><DyGraph className = "blackie" name = "temp2" parentData = {chart1}/></div>
    </div>
    <div className='row h-50'>
      <div id = "mapka"><Ymap/></div>
      <div id = "third" className = "blackie col d-flex flex-column justify-content-around"><DyGraph name = "temp" parentData = {chart1}/></div>
      <div id = "fourth" className = "blackie col d-flex flex-column justify-content-around"><DyGraph className = "blackie" name = "accel" parentData = {chart1}/></div>
    </div>
    <div style = {{height: "500px"}}>

    </div>
    </UserContext.Provider>
    </div>
  )
}
root.render(<SubApp/>);
//<Canvas>  
//<Model position={[0, 0, 0]} />
//</Canvas>,

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals();
