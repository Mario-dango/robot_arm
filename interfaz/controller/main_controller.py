"""
CONTROLADOR PRINCIPAL (El Orquestador)
Este archivo inicializa la aplicación y delega las tareas específicas 
a sus 4 sub-controladores (Managers).
"""

import os
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMessageBox

# Importamos las capas principales desde sus respectivas carpetas
from model.model import Model
from view.view import View

# Importaremos los Managers paso a paso (Por ahora están comentados para no dar error)
from .connection_manager import ConnectionManager
from .movement_manager import MovementManager
from .learning_manager import LearningManager
from .execution_manager import ExecutionManager

class MainController:
    def __init__(self):
        # 1. Inicializamos las capas MVC principales
        self.model = Model()
        self.view = View()

        # 2. --- ESTADO GLOBAL COMPARTIDO (Shadow Registers) ---
        # Todos los managers leerán y modificarán estas variables para saber dónde está el robot
        self.current_pos = {'x': 0, 'y': 0, 'z': 0}
        self.gripper_state = 'A' # A = Abierto, C = Cerrado
        self.estop_active = False # Espejo del bloqueo E-STOP del robot (campo E: de la telemetría)

        # 3. --- GESTIÓN DE RUTAS BASE ---
        base_path = os.getcwd()
        self.routines_path = os.path.join(base_path, "rutinas")
        if not os.path.exists(self.routines_path):
            try:
                os.makedirs(self.routines_path)
            except OSError as e:
                print(f"Error creando carpeta rutinas: {e}")

        # 4. --- TIMERS GLOBALES DE INTERFAZ ---
        # Timer para efecto blink del LED Finish
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.handle_finish_blink)
        self.blink_count = 0

        # Timer para parpadeo de alerta (HOME necesario)
        self.alert_timer = QTimer()
        self.alert_timer.timeout.connect(self.handle_home_alert_blink)
        self.alert_blink_state = False
        self.homing_failed = False # True si el firmware reportó un error de homing

        # 5. --- INICIALIZACIÓN DE MANAGERS ---
        # Aquí le pasamos 'self' (este controlador entero) a cada manager.
        # Así, los managers podrán acceder a app.view, app.model y app.current_pos.
        
        self.connection_mgr = ConnectionManager(self)
        self.movement_mgr = MovementManager(self)
        self.learning_mgr = LearningManager(self)
        self.execution_mgr = ExecutionManager(self)

        # 6. Conexiones generales (Ayuda y UI) que no pertenecen a ningún manager en particular
        self.init_general_ui()
        
        # Mostramos la ventana al terminar de configurar todo
        self.view.show()

    def init_general_ui(self):
        """Conecta los botones de la barra de menú superior"""
        self.view.action_manual.triggered.connect(self.show_manual)
        self.view.action_about.triggered.connect(self.show_about)
        self.view.action_kawaii.toggled.connect(self.view.toggle_kawaii_mode)

        # Ayuda de comandos del firmware (2 capas, se ve por consola)
        self.view.action_cmd_list.triggered.connect(self.show_command_list)
        self.view.action_cmd_example.triggered.connect(self.show_command_example)

        # Panel de Test / Diagnóstico
        self.view.action_test_panel.triggered.connect(self.open_test_panel)
        self._test_panel = None

    # --- AYUDA DE COMANDOS (pide al firmware que imprima por consola) ---
    def show_command_list(self):
        """CAPA 1: pide al STM32 la lista completa de comandos (:-?)."""
        self.connection_mgr.log_console("INFO", "Solicitando lista de comandos al STM32 (:-?)…")
        self.connection_mgr.send_command(":-?")

    def show_command_example(self):
        """CAPA 2: elige un comando y pide su ejemplo (:-?<Letra>)."""
        from PyQt5.QtWidgets import QInputDialog
        items = [
            "H — Homing", "Z — Set Zero", "A — Apertura garra", "P — Cierre garra",
            "E — Enable motores", "V — Velocidad global", "S — Stop (E-STOP)",
            "R — Rearmar", "I — Handshake", "# — Mover (ejecución)",
            "M — Test motor", "L — Test LED", "G — Test garra",
        ]
        eleccion, ok = QInputDialog.getItem(
            self.view, "Ejemplo de comando",
            "Elegí el comando para ver un ejemplo en la consola:", items, 0, False)
        if ok and eleccion:
            letra = eleccion.split(" ")[0]  # el primer token es la letra/símbolo
            self.connection_mgr.log_console("INFO", f"Solicitando ejemplo de '{letra}' (:-?{letra})…")
            self.connection_mgr.send_command(f":-?{letra}")

    # --- PANEL DE TEST / DIAGNÓSTICO ---
    def open_test_panel(self):
        """Abre (o trae al frente) la ventana de test de hardware."""
        from view.test_panel import TestPanel
        if self._test_panel is None:
            self._test_panel = TestPanel(self.connection_mgr.send_command, parent=self.view)
        self._test_panel.show()
        self._test_panel.raise_()
        self._test_panel.activateWindow()

    # --- SINCRONIZACIÓN DE PARO DE EMERGENCIA (robot -> interfaz) ---
    def handle_estop_engaged(self):
        """El robot entró en PARO (botón físico o :-S). Frenamos la secuencia de la
        interfaz aunque el paro no haya salido de ella, y resaltamos 'Rearmar'."""
        # 1. Frenar la ejecución automática si estaba corriendo
        if hasattr(self, 'execution_mgr'):
            self.execution_mgr.halt_by_estop()
        # 2. Avisar por consola
        if hasattr(self, 'connection_mgr'):
            self.connection_mgr.log_console("ALERTA", "PARO DE EMERGENCIA ACTIVO. Secuencia detenida. Enviá :-R (Rearmar) para continuar.")
        # 3. Resaltar el botón Rearmar (parpadeo llamativo vía estilo)
        self.view.btn_rearm.setStyleSheet(
            "background-color: #d32f2f; color: white; font-weight: bold; border: 2px solid #ff5252;")
        self.view.btn_rearm.setText("¡REARMAR! (:-R)")
        # 4. Marcar el badge WAIT como bloqueo
        self.view.lbl_status_wait.setText("E-STOP")

    def handle_estop_cleared(self):
        """El robot salió del PARO (tras :-R). Restauramos la interfaz."""
        if hasattr(self, 'connection_mgr'):
            self.connection_mgr.log_console("INFO", "Paro liberado. Recordá recalibrar (Home) antes de operar.")
        self.view.btn_rearm.setStyleSheet("")
        self.view.btn_rearm.setText("Rearmar (:-R)")
        self.view.lbl_status_wait.setText("WAIT / BUSY")

    # --- FUNCIONES DE ALERTAS VISUALES GLOBALES ---
    def handle_finish_blink(self):
        """Hace parpadear el LED de FINISH al terminar una rutina"""
        self.blink_count += 1
        if self.blink_count % 2 != 0:
            self.view.lbl_status_finish.setProperty("class", "status_badge_finish_on")
        else:
            self.view.lbl_status_finish.setProperty("class", "status_badge_off")
            
        self.view.lbl_status_finish.style().unpolish(self.view.lbl_status_finish)
        self.view.lbl_status_finish.style().polish(self.view.lbl_status_finish)
        
        if self.blink_count >= 6:
            self.blink_timer.stop()
            self.view.lbl_status_finish.setProperty("class", "status_badge_off")
            self.view.lbl_status_finish.style().unpolish(self.view.lbl_status_finish)
            self.view.lbl_status_finish.style().polish(self.view.lbl_status_finish)

    def start_home_alert(self):
        """Arranca el parpadeo del indicador HOME (robot sin calibrar)."""
        if not self.alert_timer.isActive():
            self.alert_timer.start(500)

    def stop_home_alert(self):
        """Detiene el parpadeo del indicador HOME (robot ya calibrado)."""
        if self.alert_timer.isActive():
            self.alert_timer.stop()
        self.alert_blink_state = False
        self.homing_failed = False # Al calibrar bien, se borra el estado de error

    def notify_homing_failed(self, msg):
        """El firmware reportó un error de homing: log en rojo + parpadeo marcado."""
        self.homing_failed = True
        if hasattr(self, 'connection_mgr'):
            self.connection_mgr.log_console("ERROR", f"FALLO DE HOMING → {msg}")
        self.start_home_alert() # Aseguramos que el indicador HOME esté parpadeando

    def notify_homing_ok(self):
        """El firmware reportó homing exitoso."""
        self.homing_failed = False
        if hasattr(self, 'connection_mgr'):
            self.connection_mgr.log_console("INFO", "Homing OK. Robot calibrado.")

    def handle_home_alert_blink(self):
        """Hace parpadear el indicador HOME. Muestra 'HOME FALLÓ' si hubo un error de
        homing, o 'REQ. HOMING' si simplemente falta calibrar."""
        self.alert_blink_state = not self.alert_blink_state
        texto = "HOME FALLO" if self.homing_failed else "REQ. HOMING"
        if self.alert_blink_state:
            self.view.lbl_status_home.setStyleSheet("background-color: #d32f2f; color: white; border-radius: 4px; border: 1px solid #ff5252;")
            self.view.lbl_status_home.setText(texto)
        else:
            self.view.lbl_status_home.setStyleSheet("background-color: #333; color: #555; border-radius: 4px; border: 1px solid #444;")
            self.view.lbl_status_home.setText(texto)

    # --- FUNCIONES DE LA BARRA DE AYUDA ---
    def show_manual(self):
        instrucciones = (
            "<b>Manual de Usuario — T.A.I.L.S.</b><br><br>"

            "<b>Conexión:</b> Elegí el puerto COM y presioná <b>Conectar</b>. Al conectar se "
            "envía el handshake (<code>:-I</code>) y el robot informa por consola el estado del "
            "LCD (dirección I2C detectada o si no responde).<br><br>"

            "<b>1. Calibración:</b> Hacé <b>Home</b> (<code>:-H</code>) antes de operar. "
            "Si el homing falla, el indicador <b>HOME</b> parpadea mostrando "
            "<b>HOME FALLÓ</b> y la consola detalla el motivo (qué eje y por qué). "
            "También podés fijar el cero actual con <b>Set Zero</b> (<code>:-Z</code>).<br>"

            "<b>2. Aprendizaje:</b> Movés el brazo con las flechas (jogging), ajustás velocidad "
            "e incremento, y con <b>Guardar Punto</b> armás la secuencia. Exportás a JSON.<br>"

            "<b>3. Ejecución:</b> Cargás la rutina JSON y presionás <b>Play</b>.<br><br>"

            "<b>Paro de Emergencia:</b> El botón <b>STOP EMERGENCIA</b> (o el pulsador físico) "
            "frena el robot y <u>engancha un bloqueo</u>. La interfaz lo detecta por telemetría: "
            "<b>detiene la secuencia automáticamente</b> y resalta el botón <b>¡REARMAR!</b>. "
            "Para salir del bloqueo, presioná <b>Rearmar</b> (<code>:-R</code>) y volvé a hacer Home.<br><br>"

            "<b>Herramientas → Panel de Test / Diagnóstico:</b> ventana para probar el hardware "
            "pieza por pieza: prender/apagar cada LED y mover cada motor de forma independiente. "
            "Ideal para verificar conexiones sin cargar una rutina.<br><br>"

            "<b>Ayuda → Comandos del STM32:</b> imprime en la consola la lista de comandos que "
            "interpreta el firmware. <b>Ayuda → Ejemplo de un comando…</b> muestra un ejemplo "
            "concreto del comando que elijas.<br><br>"

            "<i>Nota: si el indicador de sistema dice 'REQ. HOMING' o 'HOME FALLÓ', andá a la "
            "pestaña de Calibración y hacé Home.</i>"
        )
        QMessageBox.information(self.view, "Manual de Usuario", instrucciones)

    def show_about(self):
        about_text = (
            "<h2>T.A.I.L.S.</h2>"
            "<p><b>Technical Articulated Intelligent Linkage System</b></p>"
            "<p>Interfaz de control robótico.</p>"
            "<p>Versión 1.0</p>"
        )
        QMessageBox.about(self.view, "Acerca de", about_text)