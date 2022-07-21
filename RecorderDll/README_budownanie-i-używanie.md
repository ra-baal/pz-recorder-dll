# Budowanie zależności i solucji VS

Pliki VS nie są dodawane do repo.
Należy najpierw utworzyć solucję i projekt VS.

Można użyć CMake GUI, ale prościej użyć komendy cmd:

```
cmake -S <your-path>/<point-cloud-recorder-dll>/RecorderDll -B <your-path>/<point-cloud-recorder-dll>/RecorderDll
```

(Obie ścieżki takie same)

a jeszcze prościej uruchomić ją ze skryptu `CMake Solution Build.cmd`.

# Używanie

Nowe pliki trzeba tworzyć ręcznie w struktruze katalogów (`source` i podkatalogi)
oraz dopisywać je w `CMakeLists.txt (add_executable)`.
Visual Studio niestety średnio współpracuje,
więc w nim dodajemy już utworzone pliku (`add -> existing items`).
Z drugiej strony VS potrafi spytać czy ma korzystać z `CMakeLists`
i wtedy np. nie zbuduje projektu jeśli jest tam wymieniony jakiś plik,
który nie istnieje.

# Opis plików i katalogów

*to-do*
