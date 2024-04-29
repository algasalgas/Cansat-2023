#![cfg_attr(
  all(not(debug_assertions), target_os = "windows"),
  windows_subsystem = "windows"
)]
use std::{time::Duration, io::{self, Write}, fmt::format};
use serialport::SerialPortInfo;
use tauri::{
  plugin::{Builder, TauriPlugin},
  Manager, Runtime,
};

use command::{available_ports, cancel_read, close, close_all, force_close, open, read, write, write_binary};
use state::SerialportState;
use std::{
  collections::HashMap,
  sync::{Arc, Mutex},
};
use futures::stream::StreamExt;
use tokio::io::AsyncWriteExt;
use std::{env, str, path::{self, Path}};
use tokio_util::codec::{Decoder, Encoder};
use tokio_serial::SerialPortBuilderExt;
use tokio::fs::File;
use std::io::prelude::*;
pub mod command;
pub mod error;
pub mod state;
pub mod test;

/// Initializes the plugin.
pub fn init<R: Runtime>() -> TauriPlugin<R> {
  Builder::new("serialport")
      .invoke_handler(tauri::generate_handler![
          available_ports,
          cancel_read,
          close,
          close_all,
          force_close,
          open,
          read,
          write,
          write_binary,
      ])
      .setup(move |app_handle| {
          app_handle.manage(SerialportState {
              serialports: Arc::new(Mutex::new(HashMap::new())),
          });
          Ok(())
      })
      .build()
}

#[tauri::command]
fn greet(name: &str) -> Vec<String> {
  let mut vec = Vec::new();
  let ports = serialport::available_ports().expect("No ports found!");
  for p in ports {
    vec.push(p.port_name);
  }
  vec
}
//{PORTNAME};115200

#[tauri::command]
fn connect(name: &str) -> String {
  format!("heeehee")
}
extern crate dirs;
#[tokio::main]
async fn main() -> tokio_serial::Result<()> {
  let path = Path::new(&dirs::config_dir().unwrap()).join("sporadic2");
  std::fs::create_dir_all(path)?;
  #[tauri::command]
  async fn update_txt(js_msg: String) {
    let path = Path::new(&dirs::config_dir().unwrap()).join("sporadic2").join("smth.txt");
    println!("{:?}", &path);
    let mut file = tokio::fs::OpenOptions::new()
        .write(true)
        .create_new(false)
        .append(true)
        .open(path)
        .await.unwrap();
    file.write_all(js_msg.as_bytes()).await.expect("No");
    println!("{}", js_msg);
     // I'm Rust!
  }
  #[tauri::command]
  async fn update_processing(js_msg: String) {
    let path = Path::new(&dirs::config_dir().unwrap()).join("sporadic2").join("sometext.txt");
    println!("{:?}", &path);
    let mut file = tokio::fs::OpenOptions::new()
        .write(true)
        .create_new(false)
        .append(true)
        .open(path)
        .await.unwrap();
    file.write_all(js_msg.as_bytes()).await.expect("No");
    println!("{}", js_msg);
     // I'm Rust!
  }
  #[tauri::command]
  async fn update_coord(js_msg: String)  {
    let path = Path::new(&dirs::config_dir().unwrap()).join("sporadic2").join("smthh.txt");
    println!("{:?}", &path);
    let mut file = tokio::fs::OpenOptions::new()
        .write(true)
        .create_new(false)
        .append(true)
        .open(path)
        .await.unwrap();
    file.write_all(js_msg.as_bytes()).await.expect("No");
    println!("{}", js_msg);
     // I'm Rust!
  }
  #[tauri::command]
  async fn madgwick_calc(js_msg: String) -> Vec<f64> {
    let mut vec = Vec::new();
      vec.push(1.);
      vec.push(1.);
      vec.push(1.);
    vec
     // I'm Rust!
  }
  tauri::Builder::default()
    .plugin(init())
    .invoke_handler(tauri::generate_handler![greet, update_coord, update_txt, madgwick_calc, update_processing])
    .run(tauri::generate_context!())
    .expect("error while running tauri application");
  Ok(())
}
