import wpilib
from ntcore import NetworkTableInstance
import constants



class VoiceReceiver:
    _last_timestamp = 0.0
    _current_command = ""
    _current_value = 0.0
    _command_consumed = True

    def setup(self) -> None:
        nt = NetworkTableInstance.getDefault()
        self.voice_table = nt.getTable(constants.KVoiceNTTable)


    def get_command(self) -> str:
        return self._current_command

    def get_value(self) -> float:
        return self._current_value

    def has_new_command(self) -> bool:
        return not self._command_consumed and self._current_command != "" and self._current_value != "none"

    def clear_command(self) -> None:
        self._command_consumed = True


    def execute(self) -> None:
        timestamp = self.voice_table.getNumber("timestamp", 0.0)
        if timestamp > self._last_timestamp:
            self._last_timestamp = timestamp
            self._current_command = self.voice_table.getString("command","none")
            self._current_value = self.voice_table.getNumber("value", 0.0)
            self._command_consumed = False



