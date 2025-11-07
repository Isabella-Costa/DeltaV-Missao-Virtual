# Missão Virtual  
**Desafio virtualizado para os setores de Visão Computacional e Controle & Simulação de um Drone Autônomo**

---

## Missão 01: Bate e Volta  

### Visão Geral da Missão
O objetivo dessa missão é projetar e validar um sistema de navegação autônoma para um Veículo Aéreo Não Tripulado (VANT), focado em **reconhecimento de padrões** e **pouso de precisão**.  

A missão adaptada iniciou com a operação do drone cercado por quatro figuras geométricas distintas (distratores e o alvo).  

A tarefa do VANT era:

- Identificar a figura-alvo correta entre os múltiplos distratores.  
- Navegar até a posição do alvo.  
- Executar um pouso de precisão sobre o centro da figura.  

---

##  Configuração de Ambiente de Simulação (WSL + ArduPilot + Webots)

Este documento detalha o processo completo para configurar um ambiente de desenvolvimento e simulação no **WSL (Ubuntu)**, incluindo:

- Python 3.10.12 (compilado da fonte)  
- ArduPilot SITL (Software In The Loop)  
- Webots (Simulador de física)  
- Mediamtx (Servidor de streaming RTSP para a câmera)  
- VS Code (IDE)  

---

##  1. Configuração do WSL (Windows Subsystem for Linux)

Começamos instalando e atualizando o WSL e o Ubuntu.

```bash
# 1. Instala o WSL com a distribuição padrão (Ubuntu)
wsl --install

# 2. (Dentro do Ubuntu) Atualiza os pacotes do sistema
sudo apt update
sudo apt upgrade -y

```


##  2. Instalação do Python 3.10 (Compilação Manual)
Instalar Dependências de Compilação
Instale as bibliotecas necessárias para compilar o Python.

``` bash
sudo apt install -y build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev curl \
libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev \
libffi-dev liblzma-dev
```
### Baixar e Extrair o Código-Fonte
``` bash
cd ~
wget https://www.python.org/ftp/python/3.10.12/Python-3.10.12.tgz
tar -xf Python-3.10.12.tgz
```

### Compilar o Python
```
cd Python-3.10.12
./configure --enable-optimizations
make -j $(nproc)
```

### Instalar e Verificar

``` 
sudo make altinstall
python3.10 --version
💡 Nota: É possível usar o pyenv como alternativa para gerenciar múltiplas versões do Python.
```

## 3. Instalação do ArduPilot SITL
### O ArduPilot é o "cérebro" do drone durante a simulação.

```
# Clonar o repositório do ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

### Inicializar e atualizar submódulos
```
git submodule update --init --recursive
```
### Instalar pré-requisitos
```
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### Recarregar o perfil
```
source ~/.profile
```

### Ativar o ambiente virtual criado pelo ArduPilot
```
source $HOME/venv-ardupilot/bin/activate
```

### Instalar bibliotecas adicionais
```
pip install dronekit dronekit-sitl empy==3.3.4
```
## 4. Instalação do Webots (Simulador)
```
# Instale o Webots (baixe o arquivo .deb no site oficial)
sudo apt install ./webots_2023b_amd64.deb
```
##5. Instalação da Câmera (Mediamtx RTSP Server)
```
O Mediamtx cria um stream RTSP compatível com o cv2.VideoCapture() do OpenCV.

```
### Baixe a versão mais recente no GitHub do Mediamtx
```
wget https://github.com/bluenviron/mediamtx/releases/download/v1.8.1/mediamtx_v1.8.1_linux_amd64.tar.gz
```
###  Extraia o arquivo
```
tar -xvzf mediamtx_v1.8.1_linux_amd64.tar.gz
```
### Mova o executável para o PATH do sistema
```
sudo mv mediamtx /usr/local/bin/
```
### Crie diretório de configuração
```
sudo mkdir -p /usr/local/etc/
```
### Mova o arquivo de configuração
```
sudo mv mediamtx.yml /usr/local/etc/
```

Para executar o servidor:
```
mediamtx
```
## 6. Instalação do VS Code (IDE)
# Baixe o arquivo .deb do site oficial e instale
```
sudo apt install ./code_1.xx.x_amd64.deb
# Abra o VS Code no diretório atual
code .
```

Importante:
No VS Code, instale a extensão WSL para se conectar corretamente ao seu ambiente Ubuntu.

7. Configuração do Projeto
Clone seu repositório dentro do Ubuntu/WSL.

```
cd /caminho/para/seu/projeto
python3.10 -m venv .venv
source .venv/bin/activate
```


Instale as dependências do projeto
```
nginx
Copiar código
numpy
opencv-python
dronekit
pymavlink
```
No VS Code, selecione o interpretador correto:
```
Ctrl + Shift + P → Python: Select Interpreter → escolha o .venv do projeto.
```
