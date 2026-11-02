# Guide use ESP IDF

## 1. Install ESP

> Solo con la extension vale ya que este trae ya un docker con todo

Install esp32 manager

``` bash
echo "deb [trusted=yes] https://dl.espressif.com/dl/eim/apt/ stable main" | sudo tee /etc/apt/sources.list.d/espressif.list
sudo apt update

# cli + gui
#sudo apt install eim

# onli cli
sudo apt install eim-cli
```

Install dependencies project for esp32 using CLI.
+ Default directory: `/home/asantos/.espressif`
+ Install components ...

``` bash
# Automatic install
eim install

# Install with selection
#eim wizard

# Install a specidifc version
#eim install -i v5.4.2
```

> https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-setup.html#offline-installation

> Esto genera eim_config.toml que es para repetir exactamente la misma instalacion sin romper nada
>   No va en el proyecto

## 2. Install extension ESP

``` bash
code --install-extension espressif.esp-idf-extension
```

OR

extensions.json
``` json
{
  "recommendations": [
    "espressif.esp-idf-extension"
  ]
}
```

> https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/index.html

OR

Select profile clangd:
> https://docs.espressif.com/projects/vscode-esp-idf-extension/zh_CN/latest/additionalfeatures/esp-idf-profiles.html

+ The url is: https://raw.githubusercontent.com/espressif/vscode-esp-idf-extension/master/profiles/esp-idf-clangd.code-profile

## 3. Start project

+ Type ESP-IDF: New Project

Select sample_project and:
+ IDF target: Son las distintas familias de chips de Espressif. Cada una es un SoC diferente, con su CPU, periféricos y radios:

| Target | CPU | Radio | Para qué |
|---|---|---|---|
| **esp32** | 2× Xtensa LX6, 240 MHz | WiFi 4 + BT clásico + BLE | El original, el tuyo |
| **esp32s2** | 1× Xtensa LX7 | WiFi, **sin BT** | USB OTG nativo, bajo coste |

**`linux`** no es un chip. Compila tu aplicación para **ejecutarla en el PC** como un proceso normal. Sirve para tests unitarios y lógica sin hardware (los componentes de FreeRTOS, NVS, etc. tienen versión simulada), sin necesidad de flashear.

## 4. Estructura del proyecto

+ En componentes estan esas partes del codigo que se comunican con el hardware.
  + Pueden ser nuestros
  + Del propio esp, que creo que no hace falta ponerlos ahi ya que estan en el propio sdk
  + De terceros, se pone como dependencia en ...

+ Esta el sdk que nos permite compilar sin dependecias
  + En el caso de vscode, este mira en .devcontainer para leer un fichero de configuracion y un dockerfile.

## 5. Compile

El `.devconatiner` contiene un docker con las dependencias y asi no haria flata inatalar esp idf.

La extension instala los settings de vscode para ajustar los controles a ella.