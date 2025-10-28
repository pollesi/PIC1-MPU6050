#include <Wire.h>  // A única biblioteca permitida!

// --- Constantes do MPU-6050 (Registradores) ---
const int MPU_ADDR = 0x68;      // Endereço I2C padrão do MPU-6050
const int PWR_MGMT_1 = 0x6B;    // Registrador de gerenciamento de energia
const int ACCEL_CONFIG = 0x1C;  // Registrador de configuração do acelerômetro
const int ACCEL_XOUT_H = 0x3B;  // Início dos dados do acelerômetro (6 bytes)

// --- Constantes de Física e Escala ---
// O sensor precisa ser configurado para o range que queremos
// Vamos configurar para ±8g (o suficiente para nosso uso
// O valor "cru" (raw) de 16 bits vai de -32768 a +32767 (2^16)
// A escala é (ValorMaximo(m/s^2) / (ValorMaximoRaw)
const float GRAVIDADE_G = 9.81;
const float ACCEL_SCALE_FACTOR_8G = (8.0 * GRAVIDADE_G) / 32768.0;

// --- Constantes do Projeto (Seus parâmetros testados) ---
const float THRESHOLD_PICO_XY = 5.0;  // Limite de aceleração para '0' (X) ou '1' (Y) --- Testado manualmente a partir do gráfico
const float THRESHOLD_PICO_Z = 6.0;   // Limite para o gesto "Enter" (Z)
const float THRESHOLD_RUIDO = 1.0;    // Nível de aceleração para considerar "repouso"

// --- Variáveis de Estado ---
String bitBuffer = "";     // Inicializa o buffer dos bits vazio
String palavraAtual = "";  // Inicializa o buffer da palavra vazio

// Define dois estados para evitar leituras desnecessárias após um gesto (Mais explicações ao longo do código)
enum Estado { AGUARDANDO,
              PROCESSANDO_PICO };
Estado estadoAtual = AGUARDANDO;  // Inicia o programa aguardando um gesto

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Inicializa a comunicação I2C (Wire.h)

  Serial.println("Iniciando o sistema de escrita binaria no ar...");

  // --- Inicialização Manual do MPU-6050 ---

  // 1. Acordar o MPU-6050 (por padrão ele vem em sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);  // Aponta para o registrador de energia
  Wire.write(0);           // Escreve 0x00 para "acordar"
  Wire.endTransmission(true);

  // 2. Configurar o range do acelerômetro para ±8g (Valor definido inicialmente)
  // 0x00 = ±2g | 0x08 = ±4g | 0x10 = ±8g | 0x18 = ±16g (Caso seja necessário aumentar o range)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);  // Aponta para o registrador de config. do acelerômetro
  Wire.write(0x10);          // Escreve 0x10 para configurar o range de ±8g
  Wire.endTransmission(true);

  // Menu inicial
  Serial.println("==================================================");
  Serial.println("Sistema pronto!");
  Serial.println("Faca um gesto rapido em +X para '0'");
  Serial.println("Faca um gesto rapido em +Y para '1'");
  Serial.println("De uma 'batidinha' em +Z para 'Enter' (decodificar)");
  Serial.println("IMPORTANTE: O sensor deve estar plano, virado para cima.");
  Serial.println("==================================================");
}

void loop() {
  // 1. LER OS DADOS DO SENSOR

  // Variáveis para os valores crus de 16 bits
  int16_t rawAccelX, rawAccelY, rawAccelZ;

  // Aponta para o primeiro registrador de dados do acelerômetro (ACCEL_XOUT_H)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);  // false = envia um "restart", mantendo a conexão

  // Pede 6 bytes de dados (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
  Wire.requestFrom(MPU_ADDR, 6, true);  // true = libera a conexão I2C após a leitura

  // Lê os bytes e combina-os (Byte Alto << 8 | Byte Baixo)
  rawAccelX = (Wire.read() << 8) | Wire.read();
  rawAccelY = (Wire.read() << 8) | Wire.read();
  rawAccelZ = (Wire.read() << 8) | Wire.read();

  // 2. CONVERTER OS DADOS CRUS (raw) PARA m/s²
  // Usamos o fator de escala que definimos (para ±8g)
  float accelX = rawAccelX * ACCEL_SCALE_FACTOR_8G;
  float accelY = rawAccelY * ACCEL_SCALE_FACTOR_8G;
  float accelZ = rawAccelZ * ACCEL_SCALE_FACTOR_8G;

  // 3. EXECUTAR A MÁQUINA DE ESTADOS
  switch (estadoAtual) {

    // ---------- ESTADO 1: AGUARDANDO ----------
    case AGUARDANDO:
      {
        bool picoX = (accelX > THRESHOLD_PICO_XY);  // Verifica se a aceleração no eixo X é suficiente para ser considerado um gesto válido
        bool picoY = (accelY > THRESHOLD_PICO_XY);  // Verifica se a aceleração no eixo Y é suficiente para ser considerado um gesto válido

        // O repouso em Z é ~9.8. O pico é (Gravidade + Threshold)
        bool picoZ = (accelZ > (GRAVIDADE_G + THRESHOLD_PICO_Z));  // Verifica se a aceleração no eixo Z é suficiente para ser considerado um gesto válido

        // Lógica de decisão:
        if (picoZ) {
          processarEnter();  // Chama a função para processar o envio do pacote de bits
          estadoAtual = PROCESSANDO_PICO;
        } else if (picoX || picoY) {
          if (accelX > accelY) {
            processarBit('0');  // Chama a função de processar bit, enviando 0 como valor
          } else {
            processarBit('1');  // Chama a função de processar bit, enviando 1 como valor
          }
          estadoAtual = PROCESSANDO_PICO;  // Altera o estado para processar as informações e evitar leituras indesejadas
        }
        break;
      }

    // ---------- ESTADO 2: PROCESSANDO_PICO ----------
    case PROCESSANDO_PICO:
      {
        bool repousoX = abs(accelX) < THRESHOLD_RUIDO;                // Verifica se o eixo X está em repouso (Aceleração menor que o ruído definido inicialmente)
        bool repousoY = abs(accelY) < THRESHOLD_RUIDO;                // Verifica se o eixo Y está em repouso (Aceleração menor que o ruído definido inicialmente)
        bool repousoZ = abs(accelZ - GRAVIDADE_G) < THRESHOLD_RUIDO;  // Verifica se o eixo Z está em repouso (Desconsiderando a gravidade)

        // Caso todos os eixos estejam em repouso, o estado é alterado para aguardar uma nova entrada
        if (repousoX && repousoY && repousoZ) {
          estadoAtual = AGUARDANDO;
        }
        break;
      }
  }  // Fim do switch-case

  delay(10);  // Pequeno delay para estabilidade (Extremamente necessário para que a lógica de alteração de estado funcione corretamente)
}

// --- Funções Auxiliares ---

// Adiciona um bit ('0' ou '1') ao buffer.
void processarBit(char bit) {
  bitBuffer += bit;                  // Adiciona o bit atual à sequência
  Serial.print("Bit adicionado: ");  // Bit atual
  Serial.print(bit);
  Serial.print("  -> Buffer atual: ");  // Sequência de bits acumulada
  Serial.println(bitBuffer);
}

// Decodifica o buffer, imprime a letra a palavra atual ou informa erros de leitura
void processarEnter() {
  Serial.println("---------------------------------");
  Serial.println("Gesto 'Enter' detectado!");

  if (bitBuffer.length() == 0) {
    Serial.println("Buffer vazio. Nenhuma letra para decodificar.");
    Serial.println("---------------------------------");
    return;
  }

  Serial.print("Binario lido: ");
  Serial.println(bitBuffer);

  char acao = decodificar(bitBuffer);  // Chama a função de decodificação e envia os bits a serem analisados 

  // Verifica se o código binário foi válido

  if (acao == '?') {
    // --- CASO DE ERRO ---
    
    // Se o código binário estava errado (ex: "1111", "000000")
    Serial.println("Erro: Codigo binario nao reconhecido.");

  } else if (acao == '\b') {
    // --- CASO DO "APAGAR" (BACKSPACE) ---
    if (palavraAtual.length() > 0) { // Caso haja caractere para ser apagado
      palavraAtual.remove(palavraAtual.length() - 1);
      Serial.println("Ação: Caractere apagado.");
    } else { // Caso NÃO haja caractere para ser apagado
      Serial.println("Ação: Não há caractere para ser apagado.");
    }
    // Imprime a palavra resultante
    Serial.print("palavra atual: ");
    Serial.println(palavraAtual);

  } else {
    // --- CASO NORMAL (LETRA ou ESPAÇO) ---

    // 1. Imprime a letra atual (ou espaço)
    Serial.print("Letra atual: ");
    Serial.println(acao);

    // 2. Adiciona a letra (ou espaço) à palavra
    palavraAtual += acao;

    // 3. Imprime a palavra completa
    Serial.print("palavra atual: ");
    Serial.println(palavraAtual);
  }

  Serial.println("---------------------------------");

  // Limpa o buffer de bits para a próxima letra
  bitBuffer = "";
}


// Converte uma String binária (ex: "01001") para uma Letra (ex: 'J').
char decodificar(String bits) {

  if (bits.equals("00000")) return ' ';   // AÇÃO: Espaço
  if (bits.equals("11111")) return '\b';  // AÇÃO: Apagar
  if (bits.equals("00001")) return 'A';
  if (bits.equals("00010")) return 'B';
  if (bits.equals("00011")) return 'C';
  if (bits.equals("00100")) return 'D';
  if (bits.equals("00101")) return 'E';
  if (bits.equals("00110")) return 'F';
  if (bits.equals("00111")) return 'G';
  if (bits.equals("01000")) return 'H';
  if (bits.equals("01001")) return 'I';
  if (bits.equals("01010")) return 'J';
  if (bits.equals("01011")) return 'K';
  if (bits.equals("01100")) return 'L';
  if (bits.equals("01101")) return 'M';
  if (bits.equals("01110")) return 'N';
  if (bits.equals("01111")) return 'O';
  if (bits.equals("10000")) return 'P';
  if (bits.equals("10001")) return 'Q';
  if (bits.equals("10010")) return 'R';
  if (bits.equals("10011")) return 'S';
  if (bits.equals("10100")) return 'T';
  if (bits.equals("10101")) return 'U';
  if (bits.equals("10110")) return 'V';
  if (bits.equals("10111")) return 'W';
  if (bits.equals("11000")) return 'X';
  if (bits.equals("11001")) return 'Y';
  if (bits.equals("11010")) return 'Z';
  // Os códigos de 11011 a 11110 estão livres.

  return '?';  // Código não reconhecido
}