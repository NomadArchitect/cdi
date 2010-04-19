/**
 * \mainpage
 * - \ref general_interface
 */

/**
 * \german
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
 * \endgerman
 */

/**
 * \english
 * \page general_interface General interface description
 *
 * This page describes general rules for the interaction between drivers and
 * the operating system which are not related to just one specific module or
 * data structure.
 *
 * The two major parts of it are conditions that the operating systems needs to
 * guarantee on the source code level (e.g. availability of certain functions)
 * and runtime conditions.
 *
 * \section gi_source Source code
 * \subsection gi_source_naming Naming
 * In order to avoid name space conflicts, the following rules should be
 * applied:
 * - Public names of the CDI implementation (i.e. globally visible symbol names
 *   and names used in CDI header files) start with cdi_ or CDI_.
 * - Public names of a CDI driver start with the driver name followed by an
 *   underscore (e.g. e1000_send_packet)
 *
 * \subsection gi_source_libc Functions of the C standard library
 * The CDI implementation doesn't have to provide a complete C standard
 * library. Drivers are limited to following parts of the library, which the
 * CDI implementation must provide:
 * - Everything in stddef.h
 * - Everything in stdint.h
 * - Everything in string.h
 * - Memory management functions in stdlib.h: malloc, calloc, realloc, free
 * - printf family in stdio.h, without fprintf and vfprintf. Additionally
 *   asprintf.
 *   \todo Further restrictions for stdio.h
 *
 * \section gi_runtime Runtime
 * \subsection gi_runtime_intr Concurrency and interruptions
 * Concurrency means (seemingly) parallel execution of code (e.g.
 * multithreading). An interruption means that running code is stopped in order
 * to execute different code, and only afterwards the stopped code is continued
 * (e.g. Unix signals).
 *
 * Both concepts require that source code is protected against unwanted effects
 * like race conditions, e.g. by locks. The respective method is OS dependent,
 * and to keep drivers simple, for CDI the following restrictions are made:
 *
 * - Driver code is never interrupted. The only exception is the function
 *   cdi_wait_irq: While it is running, IRQ handler may be executed.
 * - The use of threading is allowed if the CDI implementation ensures that for
 *   each device only one function is executed at the same time (e.g. by
 *   putting a lock in cdi_device). Drivers may not assume that threading is
 *   used, it is optional for the CDI implementation.
 * \endenglish
 */
