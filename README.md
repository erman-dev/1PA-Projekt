# Webots 1PA-Projekt - Ketchup house
### Autoři: Karel Hejl, Roman Krček


Tento robot je určen pro účast se v soutěži Ketchup House. Principem je posbírat co nejvíce plechovek kečupu v limitovaném čase. Kompletní pravidla najdete [zde](https://github.com/Erman2/1PA-Projekt/blob/master/docs/Ketchup%20house%20rules%202020.pdf).

Lokalizace plechovek je zajištěna pomocí dálkového senzoru a kompasu. Data ze senzoru a kompasu se zpracují a z těch robot vyhodnotí tři nejbližší plechovky. Pomocí goniometrických funkci vypočítá jejich polohu a vyrazí je sebrat.

Pro navigaci robot využívá dva infrared senzory k detekci čar umístěné hned vedle kol a kompas. K prevenci srážky s robotem protivníka využíváme další distance senzory.

Souřadnicový systém používaný k navigaci začíná v levém horním rohu mřížky souřadnicemi `[1,1]` a končí v pravém dolním rohu souřadnicemi `[7,7]`

[Odkaz na Youtube video popisující robota](https://youtu.be/KiVvE5tHudo)

![Playing field top view](https://github.com/Erman2/1PA-Projekt/blob/master/docs/world_top_view.png)

## Controller
Kyždý robot má svůj controller, ten obsahuje základní nastavení robota. Pomocí tohoto nastavení pak vytvoří objekt `can_bot` třídy `Canbot` která obsahuje funkce k ovládání robota.  

```matlab
storage_positions = [7 1;
                     7 2;
                     7 7;
                     7 6];
					 
robot_position = [7 4];

scan_angle = [65 300];

can_bot = CanBot('motor_left', 'motor_right', 'dst_front_can', ...
				'dst_front_bot', 'dst_left_bot', 'dst_right_bot', ...
				'compass', 'infra_left', 'infra_right', ...
				robot_position, storage_positions, scan_angle, ...
				32);
```

- `storage_positions` obsahuje souřadnice pozic, na které má robot ukládat plechovky
- `robot_position` je startovní pozice robota
- `scan_angle` je rozmezí úhlů, ve které bude robot hledat plechovky
- `can_bot` je objekt třídy `CanBot`, používá se dál v programu k práci s robotem


```matlab
while true
	
	cans = can_bot.scan_cans()

	if isempty(cans)
		wb_console_print(sprintf('No more cans to pick up'), WB_STDOUT);
		can_bot.align(can_bot.default_alignment);
		break;
	end

	for i = 1:size(cans, 1)
		target_coords = cans(i, :);
		can_bot.go_coordinates(target_coords);
	end
	
	can_bot.store_cans()
  
end
```
V této smyčce se robot neustále snaží najít nové plechovky, sesbírat je a odvézt je domů. 

K hledání plechovek slouží funkce `can_bot.scan_cans()`, která vrátí souřadnice tří nejbližších plechovek.

Tyto tři souřadnice se postupně pošlou do funkce `can_bot.go_coordinates()`, která robota navede na dané souřadnice.

Až robot posbírá všechny tři plechovky, zavolá se funkce `can_bot.store_cans()`, ta robota navede na volné místo na domovské čáře, kde plechovky vyloží a s robotem zacouvá na jeho výchozí pozici.

Jestliže ale funkce `can_bot.scan_cans()` nevrátí žádně souřadnice, smyčka se ukončí.


```matlab
while wb_robot_step(64) ~= -1

end
```
Tato smyčka zajití, že se simulace neukončí hned potom, co pouze jeden z robotů ukončí sbírání.

## Poznámky
Roboti mají nastavenou velmi malou rychlost, protože plechovky se při vyšších rychlostech přestávají řídit fyzikálními zákony a začínají všude lítat. Doporučené je proto simulaci spoštět v zrychleném módu.
