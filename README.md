# mauaboards

- MAUA_Library
  - É o conjunto de arquivos que compõem a "biblioteca MAUA". Como não é uma biblioteca instalável os arquivos devem ser inseridos na pasta do sketch 

- Esp32_v9.ino
  - É um projeto .ino para verificar a instalação da placa e a compilação utilizando a ESP32_Maua

## Instalação do pacote
- Inserir https://raw.githubusercontent.com/smartcampusmaua/mauaboards/main/mauaboards_v01/1.0.10/package_mauaboards_index.json em Arquivo>Preferências>URLs Adicionais para Gerenciadores de Placas na IDE do Arduino

#### Atualmente existe um bug de criação de pastas onde o arquivo boards.txt é inserido no local errado:
C:\Users\USUARIO\AppData\Local\Arduino15\packages\MauaBoards\hardware\esp32\1.0.10\1.0.10
  |- esp32
    |- 1.0.10
      |- 1.0.10
        |- boards.txt
        |- ...
    
É necessário mover todos os arquivos desta pasta para C:\Users\USUARIO\AppData\Local\Arduino15\packages\MauaBoards\hardware\esp32\1.0.10
  |- esp32
    |- 1.0.10
      |- boards.txt
      |- ...
