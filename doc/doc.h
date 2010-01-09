/**
 * \mainpage
 * - \ref general_interface
 */

/**
 * \page general_interface Allgemeine Schnittstellenbeschreibung
 *
 * Diese Seite beschreibt allgemeine Regeln für das Zusammenspiel von Treibern
 * und Betriebssystem, die nicht einem bestimmtem Modul oder einer bestimmten
 * Datenstruktur zugeordnet werden können.
 *
 * Die zwei wesentlichen Teile sind Bedingungen, die das Betriebssystem auf
 * Quellcode-Ebene garantieren muss (z.B. Bereitstellung von bestimmten
 * Funktionen) und Bedingungen, die zur Laufzeit gelten.
 *
 * \section gi_source Quellcode-Ebene
 * \subsection gi_source_naming Namensregeln
 * Um Namenskonflikte zu vermeiden, sind folgende Regeln zu beachten:
 * - Öffentliche Namen der CDI-Implementierung (d.h. global sichtbare Namen
 *   und in CDI-Headerdateien verwendete Namen) beginnen mit cdi_ oder CDI_.
 * - Öffentliche Namen eines CDI-Treibers beginnen mit dem Treibernamen
 *   gefolgt von einem Unterstrich (z.B. e1000_send_packet)
 *
 * \subsection gi_source_libc Funktionen der C-Standardbibliothek
 * Die CDI-Implementierung muss keine vollständige C-Standardbibliothek zur
 * Verfügung stellen. Treiber müssen sich auf folgende Teile beschränken
 * (oder die Benutzung weiterer Funktionen durch \#ifdef optional machen):
 * - Alles aus stddef.h
 * - Alles aus stdint.h
 * - Alles aus string.h
 * - Speicherverwaltungsfunktionen aus stdlib.h: malloc, calloc, realloc, free
 * - printf-Familie aus stdio.h, ohne fprintf und vfprintf. Außerdem asprintf.
 *   \todo Gebrauch von stdio.h einschränken
 *
 * \section gi_runtime Laufzeit
 * \subsection gi_runtime_intr Nebenläufigkeit und Unterbrechungen
 * Nebenläufigkeit ist das gleichzeitige oder quasi-gleichzeitige Ablaufen von
 * Code (z.B. Multithreading). Eine Unterbrechung bedeutet, dass ausgeführter
 * Code angehalten wird, um anderen Code auszuführen, und erst anschließend
 * der angehaltene Code wieder fortgesetzt wird (z.B. Unix-Signale).
 *
 * Die Nutzung beider Konzepte erfordert es, dass der Quellcode sich gegen
 * unerwünschte Effekt wie Race Conditions schützt, z.B. durch Locks. Da die
 * jeweiligen Methoden OS-abhängig sind und um Treiber einfach zu halten,
 * gelten für CDI die folgenden Einschränkungen:
 *
 * - Treibercode wird niemals unterbrochen. Die einzige Ausnahme ist die
 *   Funktion cdi_wait_irq, während der IRQ-Handler ausgeführt werden
 *   dürfen.
 * - Nebenläufigkeit ist erlaubt, sofern die CDI-Implementierung sicherstellt,
 *   dass für jedes Gerät gleichzeitig nur eine Funktion ausgeführt wird
 *   (z.B. durch ein Lock auf das cdi_device). Treiber dürfen nicht davon
 *   ausgehen, dass irgendwelche Funktionen nebenläufig ausgeführt werden.
 *   Nebenläufigkeit ist für die Implementierung optional.
 */
