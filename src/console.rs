use bevy::{
    app::AppLabel,
    ecs::{
        component::Tick,
        schedule::IntoSystemConfigs,
        system::{Resource, SystemMeta, SystemParam},
        world::unsafe_world_cell::UnsafeWorldCell,
    },
};
use bevy::{input::keyboard::KeyboardInput, prelude::*};
use bevy_egui::egui::{
    self, popup_below_widget, Align, Layout, Pos2, Rect, RichText, ScrollArea, TextEdit, Vec2,
};
use bevy_egui::egui::{text::LayoutJob, text_edit::CCursorRange};
use bevy_egui::egui::{Context, Id};
use bevy_egui::{
    egui::{epaint::text::cursor::CCursor, Color32, FontId, TextFormat},
    EguiContexts,
};
use clap::{builder::StyledStr, CommandFactory, FromArgMatches};
use fuzzy_matcher::{skim::SkimMatcherV2, FuzzyMatcher};
use shlex::Shlex;
use std::marker::PhantomData;
use std::mem;
use std::{
    cmp::Reverse,
    collections::{BTreeMap, VecDeque},
};

use crate::ConsoleSet;

type ConsoleCommandEnteredReaderSystemParam = EventReader<'static, 'static, ConsoleCommandEntered>;

type PrintConsoleLineWriterSystemParam = EventWriter<'static, PrintConsoleLine>;

/// A super-trait for command like structures
pub trait Command: NamedCommand + CommandFactory + FromArgMatches + Sized + Resource {}
impl<T: NamedCommand + CommandFactory + FromArgMatches + Sized + Resource> Command for T {}

/// Trait used to allow uniquely identifying commands at compile time
pub trait NamedCommand {
    /// Return the unique command identifier (same as the command "executable")
    fn name() -> &'static str;
}

/// Executed parsed console command.
///
/// Used to capture console commands which implement [`CommandName`], [`CommandArgs`] & [`CommandHelp`].
/// These can be easily implemented with the [`ConsoleCommand`](bevy_console_derive::ConsoleCommand) derive macro.
///
/// # Example
///
/// ```
/// # use bevy_console::ConsoleCommand;
/// # use clap::Parser;
/// /// Prints given arguments to the console.
/// #[derive(Parser, ConsoleCommand)]
/// #[command(name = "log")]
/// struct LogCommand {
///     /// Message to print
///     msg: String,
///     /// Number of times to print message
///     num: Option<i64>,
/// }
///
/// fn log_command(mut log: ConsoleCommand<LogCommand>) {
///     if let Some(Ok(LogCommand { msg, num })) = log.take() {
///         log.ok();
///     }
/// }
/// ```
pub struct ConsoleCommand<'w, T> {
    command: Option<Result<T, clap::Error>>,
    console_line: EventWriter<'w, PrintConsoleLine>,
}

impl<'w, T> ConsoleCommand<'w, T> {
    /// Returns Some(T) if the command was executed and arguments were valid.
    ///
    /// This method should only be called once.
    /// Consecutive calls will return None regardless if the command occurred.
    pub fn take(&mut self) -> Option<Result<T, clap::Error>> {
        mem::take(&mut self.command)
    }

    /// Print `[ok]` in the console.
    pub fn ok(&mut self) {
        self.console_line.send(PrintConsoleLine::new("[ok]".into()));
    }

    /// Print `[failed]` in the console.
    pub fn failed(&mut self) {
        self.console_line
            .send(PrintConsoleLine::new("[failed]".into()));
    }

    /// Print a reply in the console.
    ///
    /// See [`reply!`](crate::reply) for usage with the [`format!`] syntax.
    pub fn reply(&mut self, msg: impl Into<StyledStr>) {
        self.console_line.send(PrintConsoleLine::new(msg.into()));
    }

    /// Print a reply in the console followed by `[ok]`.
    ///
    /// See [`reply_ok!`](crate::reply_ok) for usage with the [`format!`] syntax.
    pub fn reply_ok(&mut self, msg: impl Into<StyledStr>) {
        self.console_line.send(PrintConsoleLine::new(msg.into()));
        self.ok();
    }

    /// Print a reply in the console followed by `[failed]`.
    ///
    /// See [`reply_failed!`](crate::reply_failed) for usage with the [`format!`] syntax.
    pub fn reply_failed(&mut self, msg: impl Into<StyledStr>) {
        self.console_line.send(PrintConsoleLine::new(msg.into()));
        self.failed();
    }
}

pub struct ConsoleCommandState<T> {
    #[allow(clippy::type_complexity)]
    event_reader: <ConsoleCommandEnteredReaderSystemParam as SystemParam>::State,
    console_line: <PrintConsoleLineWriterSystemParam as SystemParam>::State,
    marker: PhantomData<T>,
}

unsafe impl<T: Command> SystemParam for ConsoleCommand<'_, T> {
    type State = ConsoleCommandState<T>;
    type Item<'w, 's> = ConsoleCommand<'w, T>;

    fn init_state(world: &mut World, system_meta: &mut SystemMeta) -> Self::State {
        let event_reader = ConsoleCommandEnteredReaderSystemParam::init_state(world, system_meta);
        let console_line = PrintConsoleLineWriterSystemParam::init_state(world, system_meta);
        ConsoleCommandState {
            event_reader,
            console_line,
            marker: PhantomData,
        }
    }

    #[inline]
    unsafe fn get_param<'w, 's>(
        state: &'s mut Self::State,
        system_meta: &SystemMeta,
        world: UnsafeWorldCell<'w>,
        change_tick: Tick,
    ) -> Self::Item<'w, 's> {
        let mut event_reader = ConsoleCommandEnteredReaderSystemParam::get_param(
            &mut state.event_reader,
            system_meta,
            world,
            change_tick,
        );
        let mut console_line = PrintConsoleLineWriterSystemParam::get_param(
            &mut state.console_line,
            system_meta,
            world,
            change_tick,
        );

        let command = event_reader.iter().find_map(|command| {
            if T::name() == command.command_name {
                let clap_command = T::command().no_binary_name(true);
                let arg_matches = clap_command.try_get_matches_from(command.args.iter());

                debug!(
                    "Trying to parse as `{}`. Result: {arg_matches:?}",
                    command.command_name
                );

                match arg_matches {
                    Ok(matches) => {
                        return Some(T::from_arg_matches(&matches));
                    }
                    Err(err) => {
                        console_line.send(PrintConsoleLine::new(err.render()));
                        return Some(Err(err));
                    }
                }
            }
            None
        });

        ConsoleCommand {
            command,
            console_line,
        }
    }
}
/// Parsed raw console command into `command` and `args`.
#[derive(Clone, Debug, Event)]
pub struct ConsoleCommandEntered {
    /// the command definition
    pub command_name: String,
    /// Raw parsed arguments
    pub args: Vec<String>,
}

/// Events to print to the console.
#[derive(Clone, Debug, Eq, Event, PartialEq)]
pub struct PrintConsoleLine {
    /// Console line
    pub line: StyledStr,
}

impl PrintConsoleLine {
    /// Creates a new console line to print.
    pub const fn new(line: StyledStr) -> Self {
        Self { line }
    }
}

/// Key for toggling the console.
#[derive(Copy, Clone)]
pub enum ToggleConsoleKey {
    /// Keycode supported by bevy_input
    KeyCode(KeyCode),
    /// Raw scan code
    ScanCode(u32),
}

/// Console configuration
#[derive(Clone, Resource)]
pub struct ConsoleConfiguration {
    /// Registered keys for toggling the console
    pub keys: Vec<ToggleConsoleKey>,
    /// Left position
    pub left_pos: f32,
    /// Top position
    pub top_pos: f32,
    /// Console height
    pub height: f32,
    /// Console width
    pub width: f32,
    /// Registered console commands
    pub commands: BTreeMap<&'static str, clap::Command>,
    /// Number of commands to store in history
    pub history_size: usize,
    ///Line prefix symbol
    pub symbol: String,
}

impl Default for ConsoleConfiguration {
    fn default() -> Self {
        Self {
            keys: vec![ToggleConsoleKey::KeyCode(KeyCode::Grave)],
            left_pos: 200.0,
            top_pos: 100.0,
            height: 400.0,
            width: 800.0,
            commands: BTreeMap::new(),
            history_size: 20,
            symbol: "$ ".to_owned(),
        }
    }
}

/// Add a console commands to Bevy app.
pub trait AddConsoleCommand {
    /// Add a console command with a given system.
    ///
    /// This registers the console command so it will print with the built-in `help` console command.
    ///
    /// # Example
    ///
    /// ```
    /// # use bevy::prelude::*;
    /// # use bevy_console::{AddConsoleCommand, ConsoleCommand};
    /// # use clap::Parser;
    /// App::new()
    ///     .add_console_command::<LogCommand, _>(log_command);
    /// #
    /// # /// Prints given arguments to the console.
    /// # #[derive(Parser, ConsoleCommand)]
    /// # #[command(name = "log")]
    /// # struct LogCommand;
    /// #
    /// # fn log_command(mut log: ConsoleCommand<LogCommand>) {}
    /// ```
    fn add_console_command<T: Command, Params>(
        &mut self,
        system: impl IntoSystemConfigs<Params>,
    ) -> &mut Self;
}

impl AddConsoleCommand for App {
    fn add_console_command<T: Command, Params>(
        &mut self,
        system: impl IntoSystemConfigs<Params>,
    ) -> &mut Self {
        let sys = move |mut config: ResMut<ConsoleConfiguration>| {
            let command = T::command().no_binary_name(true);
            // .color(clap::ColorChoice::Always);
            let name = T::name();
            if config.commands.contains_key(name) {
                warn!(
                    "console command '{}' already registered and was overwritten",
                    name
                );
            }
            config.commands.insert(name, command);
        };

        self.add_systems(Startup, sys)
            .add_systems(Update, system.in_set(ConsoleSet::Commands))
    }
}

/// Console open state
#[derive(Default, Resource)]
pub struct ConsoleOpen {
    /// Console open
    pub open: bool,
}

#[derive(Resource)]
pub(crate) struct ConsoleState {
    pub(crate) buf: String,
    pub(crate) scrollback: Vec<StyledStr>,
    pub(crate) history: VecDeque<StyledStr>,
    pub(crate) history_index: usize,
    pub(crate) autocomplete_selected_index: Option<(usize, String)>,
    pub(crate) autocomplete_popup: bool,
}

impl Default for ConsoleState {
    fn default() -> Self {
        ConsoleState {
            buf: String::default(),
            scrollback: Vec::new(),
            history: VecDeque::from([StyledStr::new()]),
            history_index: 0,
            autocomplete_selected_index: None,
            autocomplete_popup: false,
        }
    }
}

pub(crate) fn console_ui(
    mut egui_context: EguiContexts,
    config: Res<ConsoleConfiguration>,
    mut keyboard_input_events: EventReader<KeyboardInput>,
    keys: Res<Input<KeyCode>>,
    mut state: ResMut<ConsoleState>,
    mut command_entered: EventWriter<ConsoleCommandEntered>,
    mut console_open: ResMut<ConsoleOpen>,
) {
    let keyboard_input_events = keyboard_input_events.iter().collect::<Vec<_>>();
    let ctx = egui_context.ctx_mut();

    let pressed = keyboard_input_events
        .iter()
        .any(|code| console_key_pressed(code, &config.keys));

    // always close if console open
    // avoid opening console if typing in another text input
    if pressed && (console_open.open || !ctx.wants_keyboard_input()) {
        console_open.open = !console_open.open;
    }

    if !console_open.open {
        return;
    }
    egui::Window::new("Console")
        .collapsible(false)
        .default_pos([config.left_pos, config.top_pos])
        .default_size([config.width, config.height])
        .resizable(true)
        .show(ctx, |ui| {
            ui.vertical(|ui| {
                let scroll_height = ui.available_height() - 80.0;

                // Scroll area
                ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .stick_to_bottom(true)
                    .max_height(scroll_height)
                    .show(ui, |ui| {
                        ui.vertical(|ui| {
                            for line in &state.scrollback {
                                let mut text = LayoutJob::default();

                                text.append(
                                    &line.to_string(), //TOOD: once clap supports custom styling use it here
                                    0f32,
                                    TextFormat::simple(FontId::monospace(14f32), Color32::GRAY),
                                );

                                ui.label(text);
                            }
                        });

                        // Scroll to bottom if console just opened
                        if console_open.is_changed() {
                            ui.scroll_to_cursor(Some(Align::BOTTOM));
                        }
                    });

                // Separator
                ui.separator();

                let possible_commands = config
                    .commands
                    .keys()
                    .copied()
                    .map(|v| v.to_string())
                    .collect::<Vec<String>>();

                let matcher = SkimMatcherV2::default().ignore_case();

                let text_edit_ui = ui.make_persistent_id("text_edit_ui");

                let mut layouter = |ui: &egui::Ui, string: &str, _wrap_width: f32| {
                    if string.is_empty() {
                        return ui.fonts(|f| f.layout_job(LayoutJob::default()));
                    }
                    let mut match_results = possible_commands
                        .iter()
                        .filter_map(|s| {
                            let score = matcher.fuzzy_indices(s, string);
                            score.map(|(score, indices)| (s, score, indices))
                        })
                        .collect::<Vec<_>>();
                    match_results.sort_by_key(|k| Reverse(k.1));

                    let mut job = LayoutJob::default();
                    if let Some((first_str, _, _)) = match_results.first() {
                        let mut it = (0..first_str.len()).peekable();
                        // // Iterate through all indices in the string
                        while let Some(j) = it.next() {
                            let start = j;
                            let end = j;
                            // let match_state = first_str.contains(&state.buf);
                            // Find all consecutive characters that have the same state
                            // Format current slice based on the state
                            let format = if j > string.len() {
                                egui::TextFormat::simple(
                                    FontId::default(),
                                    egui::Color32::from_gray(80),
                                )
                            } else {
                                egui::TextFormat::default()
                            };

                            let slice = &first_str[start..=end];
                            job.append(slice, 0.0, format);
                        }
                    } else {
                        job.append(string, 0.0, egui::TextFormat::default());
                    }

                    set_cursor_pos(ui.ctx(), text_edit_ui, string.len());
                    ui.fonts(|f| f.layout_job(job))
                };

                // Input
                let text_edit = TextEdit::singleline(&mut state.buf)
                    .layouter(&mut layouter)
                    .id(text_edit_ui)
                    .desired_width(f32::INFINITY)
                    .lock_focus(true)
                    .cursor_at_end(false)
                    .font(egui::TextStyle::Monospace);

                let text_edit_response = ui.add(text_edit);

                let popup_id = ui.make_persistent_id("text_edit_popup");
                egui::popup::popup_below_widget(ui, popup_id, &text_edit_response, |ui| {
                    let mut match_results = possible_commands
                        .iter()
                        .filter_map(|s| {
                            let score = matcher.fuzzy_indices(s, &state.buf);
                            score.map(|(score, indices)| (s, score, indices))
                        })
                        .collect::<Vec<_>>();
                    match_results.sort_by_key(|k| Reverse(k.1));
                    if !match_results.is_empty() {
                        for (suggestion_index, suggestion) in match_results.iter().enumerate() {
                            if let Some(index) = state.autocomplete_selected_index.as_ref() {
                                if suggestion_index == index.0 {
                                    ui.label(
                                        RichText::new(format!(
                                            "{}: {}",
                                            suggestion.0, suggestion.1
                                        ))
                                        .color(Color32::from_rgb(255 / 2, 255 / 2, 255)),
                                    );
                                    state.autocomplete_selected_index =
                                        Some((suggestion_index, suggestion.0.clone()));
                                } else {
                                    ui.label(
                                        RichText::new(format!(
                                            "{}: {}",
                                            suggestion.0, suggestion.1
                                        ))
                                        .color(Color32::from_rgb(255 / 2, 255 / 2, 255 / 2)),
                                    );
                                }
                            } else {
                                ui.label(
                                    RichText::new(format!("{}: {}", suggestion.0, suggestion.1))
                                        .color(Color32::from_rgb(255, 255, 255)),
                                );
                            }
                        }
                    } else {
                        ui.label("❌ No suggestions");
                    }
                });

                if text_edit_response.lost_focus()
                    && ui.input(|i| i.key_pressed(egui::Key::Enter))
                    && state.autocomplete_selected_index.is_none()
                {
                    if state.buf.trim().is_empty() {
                        state.scrollback.push(StyledStr::new());
                    } else {
                        let msg = format!("{}{}", config.symbol, state.buf);
                        state.scrollback.push(msg.into());
                        let cmd_string = state.buf.clone();
                        state.history.insert(1, cmd_string.into());
                        if state.history.len() > config.history_size + 1 {
                            state.history.pop_back();
                        }

                        let mut args = Shlex::new(&state.buf).collect::<Vec<_>>();

                        if !args.is_empty() {
                            let command_name = args.remove(0);
                            debug!("Command entered: `{command_name}`, with args: `{args:?}`");

                            let command = config.commands.get(command_name.as_str());

                            if command.is_some() {
                                command_entered.send(ConsoleCommandEntered { command_name, args });
                            } else {
                                debug!(
                                    "Command not recognized, recognized commands: `{:?}`",
                                    config.commands.keys().collect::<Vec<_>>()
                                );

                                state.scrollback.push("error: Invalid command".into());
                            }
                        }

                        state.buf.clear();

                        state.autocomplete_selected_index = None;
                        ui.memory_mut(|mem| {
                            if mem.is_popup_open(popup_id) {
                                mem.close_popup()
                            }
                        });
                    }
                }

                if text_edit_response.lost_focus()
                    && ui.input(|i| i.key_pressed(egui::Key::Enter))
                    && state.autocomplete_selected_index.is_some()
                {
                    state.buf = state
                        .autocomplete_selected_index
                        .as_ref()
                        .unwrap()
                        .1
                        .clone();

                    state.autocomplete_popup = false;
                    // state.autocomplete_selected_index = None;
                }

                if text_edit_response.has_focus()
                    && state.buf.len() != 0
                    && state.buf.len()
                        != state
                            .autocomplete_selected_index
                            .as_ref()
                            .unwrap_or(&(0, String::new()))
                            .1
                            .len()
                {
                    state.autocomplete_popup = true;
                    ui.memory_mut(|mem| mem.open_popup(popup_id));
                } else {
                    state.autocomplete_popup = false;
                    ui.memory_mut(|mem| {
                        if mem.is_popup_open(popup_id) {
                            mem.close_popup()
                        }
                    });
                }
                // Clear on ctrl+l
                if keyboard_input_events
                    .iter()
                    .any(|&k| k.state.is_pressed() && k.key_code == Some(KeyCode::L))
                    && (keys.any_pressed([KeyCode::ControlLeft, KeyCode::ControlRight]))
                {
                    state.autocomplete_popup = false;
                    state.scrollback.clear();
                }

                if state.autocomplete_popup {
                    let current_selected_value = state.autocomplete_selected_index.as_ref();
                    // Handle up and down through suggestions
                    if text_edit_response.has_focus()
                        && ui.input(|i| i.key_pressed(egui::Key::ArrowUp))
                    {
                        if state.autocomplete_selected_index.is_none() {
                            state.autocomplete_selected_index = Some((0, String::new()));
                        } else {
                            let current_value = current_selected_value.unwrap().0;
                            if current_value == 0 {
                                state.autocomplete_selected_index = None;
                            } else {
                                state.autocomplete_selected_index =
                                    Some((current_value.saturating_sub(1), String::new()));
                            }
                        }
                    } else if text_edit_response.has_focus()
                        && ui.input(|i| i.key_pressed(egui::Key::ArrowDown))
                    {
                        if state.autocomplete_selected_index.is_none() {
                            state.autocomplete_selected_index = Some((0, String::new()));
                        } else {
                            let current_value = current_selected_value.unwrap().0;
                            state.autocomplete_selected_index =
                                Some((current_value.saturating_add(1), String::new()));
                        }
                    }
                } else {
                    ui.memory_mut(|mem| {
                        if mem.is_popup_open(popup_id) {
                            mem.close_popup()
                        }
                    });
                    // Handle up and down through history
                    if text_edit_response.has_focus()
                        && ui.input(|i| i.key_pressed(egui::Key::ArrowUp))
                        && state.history.len() > 1
                        && state.history_index < state.history.len() - 1
                    {
                        if state.history_index == 0 && !state.buf.trim().is_empty() {
                            *state.history.get_mut(0).unwrap() = state.buf.clone().into();
                        }

                        state.history_index += 1;
                        let previous_item = state.history.get(state.history_index).unwrap().clone();
                        state.buf = previous_item.to_string();

                        set_cursor_pos(ui.ctx(), text_edit_response.id, state.buf.len());
                    } else if text_edit_response.has_focus()
                        && ui.input(|i| i.key_pressed(egui::Key::ArrowDown))
                        && state.history_index > 0
                    {
                        state.history_index -= 1;
                        let next_item = state.history.get(state.history_index).unwrap().clone();
                        state.buf = next_item.to_string();

                        set_cursor_pos(ui.ctx(), text_edit_response.id, state.buf.len());
                    }
                }

                // Focus on input
                ui.memory_mut(|m| m.request_focus(text_edit_response.id));
            });
        });
}

pub(crate) fn receive_console_line(
    mut console_state: ResMut<ConsoleState>,
    mut events: EventReader<PrintConsoleLine>,
) {
    for event in events.iter() {
        let event: &PrintConsoleLine = event;
        console_state.scrollback.push(event.line.clone());
    }
}

fn console_key_pressed(
    keyboard_input: &KeyboardInput,
    configured_keys: &[ToggleConsoleKey],
) -> bool {
    if !keyboard_input.state.is_pressed() {
        return false;
    }

    for configured_key in configured_keys {
        match configured_key {
            ToggleConsoleKey::KeyCode(configured_key_code) => match keyboard_input.key_code {
                None => continue,
                Some(pressed_key) => {
                    if configured_key_code == &pressed_key {
                        return true;
                    }
                }
            },
            ToggleConsoleKey::ScanCode(configured_scan_code) => {
                if &keyboard_input.scan_code == configured_scan_code {
                    return true;
                }
            }
        }
    }

    false
}

fn set_cursor_pos(ctx: &Context, id: Id, pos: usize) {
    if let Some(mut state) = TextEdit::load_state(ctx, id) {
        state.set_ccursor_range(Some(CCursorRange::one(CCursor::new(pos))));
        state.store(ctx, id);
    }
}

#[cfg(test)]
mod tests {
    use bevy::input::ButtonState;

    use super::*;

    #[test]
    fn test_console_key_pressed_scan_code() {
        let input = KeyboardInput {
            scan_code: 41,
            key_code: None,
            state: ButtonState::Pressed,
            window: Entity::PLACEHOLDER,
        };

        let config = vec![ToggleConsoleKey::ScanCode(41)];

        let result = console_key_pressed(&input, &config);
        assert!(result);
    }

    #[test]
    fn test_console_wrong_key_pressed_scan_code() {
        let input = KeyboardInput {
            scan_code: 42,
            key_code: None,
            state: ButtonState::Pressed,
            window: Entity::PLACEHOLDER,
        };

        let config = vec![ToggleConsoleKey::ScanCode(41)];

        let result = console_key_pressed(&input, &config);
        assert!(!result);
    }

    #[test]
    fn test_console_key_pressed_key_code() {
        let input = KeyboardInput {
            scan_code: 0,
            key_code: Some(KeyCode::Grave),
            state: ButtonState::Pressed,
            window: Entity::PLACEHOLDER,
        };

        let config = vec![ToggleConsoleKey::KeyCode(KeyCode::Grave)];

        let result = console_key_pressed(&input, &config);
        assert!(result);
    }

    #[test]
    fn test_console_wrong_key_pressed_key_code() {
        let input = KeyboardInput {
            scan_code: 0,
            key_code: Some(KeyCode::A),
            state: ButtonState::Pressed,
            window: Entity::PLACEHOLDER,
        };

        let config = vec![ToggleConsoleKey::KeyCode(KeyCode::Grave)];

        let result = console_key_pressed(&input, &config);
        assert!(!result);
    }

    #[test]
    fn test_console_key_right_key_but_not_pressed() {
        let input = KeyboardInput {
            scan_code: 0,
            key_code: Some(KeyCode::Grave),
            state: ButtonState::Released,
            window: Entity::PLACEHOLDER,
        };

        let config = vec![ToggleConsoleKey::KeyCode(KeyCode::Grave)];

        let result = console_key_pressed(&input, &config);
        assert!(!result);
    }
}
