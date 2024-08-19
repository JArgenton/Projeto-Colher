from flask import Flask, jsonify
import requests
from flask_cors import CORS

import pandas as pd

from datetime import datetime
import signal
import sys

app = Flask(__name__)
CORS(app)  # Habilita CORS para todas as rotas

# Inicializa um DataFrame global para armazenar os dados
df = pd.DataFrame(columns=['tempo', 'pitch', 'roll'])

@app.route('/data', methods=['GET'])
def get_data():
    """Tenta pegar dados do ESP32"""
    global df  # Indica que estamos usando o DataFrame global
    
    try:
        # Faz uma requisição GET ao ESP32 para obter os dados, substitua pelo IP do ESP32
        response = requests.get('http://192.168.0.185/data', timeout=5)

        # Converte a resposta da requisição (que deve ser um JSON) em um objeto Python
        data = response.json()

        # Obtém os valores de pitch, roll e yaw do JSON recebido, retornando 0 se algum deles não estiver presente
        pitch = data.get('pitch', 0)
        roll = data.get('roll', 0)
        yaw = data.get('yaw', 0)

        # Cria um novo DataFrame com os dados recebidos
        novo_dado = pd.DataFrame([{'tempo': datetime.now(), 'pitch': pitch, 'roll': roll}])
        
        # Adiciona os novos dados ao DataFrame global usando pd.concat
        global df
        df = pd.concat([df, novo_dado], ignore_index=True)

        # Retorna os dados como uma resposta JSON para o cliente que fez a solicitação ao servidor Flask
        return jsonify({
            'pitch': pitch,
            'roll': roll,
            'yaw': yaw
        })
    except requests.exceptions.RequestException as e:
        # Em caso de erro na comunicação com o ESP32, imprime o erro e retorna uma resposta com valores zerados
        print(f"Erro na comunicação com o ESP32: {e}")
        return jsonify({'pitch': 0, 'roll': 0, 'yaw': 0})

def salvar_df_excel(sig, frame):
    """Função para salvar o DataFrame em um arquivo Excel ao encerrar o programa"""
    global df
    df.to_excel("dados_pitch_roll.xlsx", index=False)
    print("DataFrame salvo no arquivo 'dados_pitch_roll.xlsx'.")
    sys.exit(0)  # Encerra o programa

# Captura o sinal de interrupção (Ctrl+C) e chama a função salvar_df_excel
signal.signal(signal.SIGINT, salvar_df_excel)

if __name__ == '__main__':
    # Inicia o servidor Flask, que estará acessível em todos os endereços IP (0.0.0.0) na porta 5000
    app.run(host='0.0.0.0', port=5000)
