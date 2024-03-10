use std::cell::RefCell;

use iced::alignment::Alignment;
use iced::executor;
use iced::mouse;
use iced::theme::Theme;
use iced::widget::canvas::{Cache, Geometry, LineCap, Path, Stroke, Style};
use iced::widget::{button, canvas, column, container, slider, text, Row};
use iced::{
    Application, Color, Command, Element, Length, Point, Rectangle, Settings, Size, Subscription,
};
use rapier2d::prelude::*;

const GROUND_WIDTH: f32 = 100.0;
const GROUND_HEIGHT: f32 = 0.1;
const GROUND_COLOR: Color = Color::from_rgb(0.5, 0.5, 0.5);
const GROUND_RESTITUTION: f32 = 0.7;

const GRAVITY: Vector<f32> = vector![0.0, -9.81];

const SCALE_FACTOR: f32 = 50.0;
const BACKGROUND_COLOR: Color = Color::WHITE;

const TIME_STEP: u64 = 16;

pub fn main() -> iced::Result {
    PhysicsSimulation::run(Settings::default())
}

struct Capsule {
    body_handle: RigidBodyHandle,
    length: f32,
    radius: f32,
    angle: f32,
    color: Color,
    restitution: f32,
    initial_height: f32,
}

struct PhysicsSimulation {
    capsules: Vec<Capsule>,
    selected_capsule_index: usize,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    physics_hooks: (),
    event_handler: (),
    cache: Cache,
    debug_text: RefCell<String>,
    is_playing: bool,
}

#[derive(Debug, Clone, Copy)]
enum Message {
    Tick,
    TogglePlayback,
    ResetSimulation,
    StepSimulation,
    CapsuleSelected(usize),
    CapsuleLengthChanged(f32),
    CapsuleRadiusChanged(f32),
    CapsuleAngleChanged(f32),
}

impl PhysicsSimulation {
    fn init_world(&mut self) {
        // iterate over all the rigid bodies and remove them
        let rigid_body_handles: Vec<_> = self
            .rigid_body_set
            .iter()
            .map(|(handle, _)| handle)
            .collect();
        for rigid_body_handle in rigid_body_handles {
            self.rigid_body_set.remove(
                rigid_body_handle,
                &mut self.island_manager,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                true,
            );
        }

        // Create the ground
        let ground_collider = ColliderBuilder::cuboid(GROUND_WIDTH, GROUND_HEIGHT)
            .restitution(GROUND_RESTITUTION)
            .build();
        self.collider_set.insert(ground_collider);

        // Create the bouncing capsules
        for capsule in &mut self.capsules {
            let capsule_body = RigidBodyBuilder::dynamic()
                .translation(vector![0.0, capsule.initial_height])
                .rotation(capsule.angle)
                .build();
            let capsule_collider =
                ColliderBuilder::capsule_x(capsule.length / 2.0, capsule.radius / 2.0)
                    .restitution(capsule.restitution)
                    .build();
            capsule.body_handle = self.rigid_body_set.insert(capsule_body);
            self.collider_set.insert_with_parent(
                capsule_collider,
                capsule.body_handle,
                &mut self.rigid_body_set,
            );
        }
    }

    fn step(&mut self) {
        self.physics_pipeline.step(
            &GRAVITY,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &self.physics_hooks,
            &self.event_handler,
        );
    }
}

impl Application for PhysicsSimulation {
    type Executor = executor::Default;
    type Message = Message;
    type Theme = Theme;
    type Flags = ();

    fn new(_flags: ()) -> (Self, Command<Message>) {
        let mut app = PhysicsSimulation {
            capsules: vec![
                Capsule {
                    body_handle: RigidBodyHandle::invalid(),
                    length: 2.0,
                    radius: 0.5,
                    angle: 0.0,
                    color: Color::from_rgb(0.0, 0.5, 0.8),
                    restitution: 0.7,
                    initial_height: 8.0,
                },
                Capsule {
                    body_handle: RigidBodyHandle::invalid(),
                    length: 4.0,
                    radius: 0.5,
                    angle: 0.0,
                    color: Color::from_rgb(0.0, 0.5, 0.8),
                    restitution: 0.7,
                    initial_height: 4.0,
                },
                // Add more capsules here...
            ],
            selected_capsule_index: 0,
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            physics_hooks: (),
            event_handler: (),
            cache: Cache::default(),
            debug_text: "".to_string().into(),
            is_playing: false,
        };

        app.init_world();

        (app, Command::none())
    }

    fn title(&self) -> String {
        String::from("Bouncing Capsules - Iced")
    }

    fn update(&mut self, message: Message) -> Command<Message> {
        match message {
            Message::Tick => {
                if self.is_playing {
                    self.step();
                    self.cache.clear();
                }
            }
            Message::TogglePlayback => {
                self.is_playing = !self.is_playing;
            }
            Message::ResetSimulation => {
                self.init_world();
                self.cache.clear();
            }
            Message::StepSimulation => {
                if !self.is_playing {
                    self.step();
                    self.cache.clear();
                }
            }
            Message::CapsuleSelected(index) => {
                self.selected_capsule_index = index;
            }
            Message::CapsuleLengthChanged(length) => {
                self.capsules[self.selected_capsule_index].length = length;
                self.init_world();
                self.cache.clear();
            }
            Message::CapsuleRadiusChanged(radius) => {
                self.capsules[self.selected_capsule_index].radius = radius;
                self.init_world();
                self.cache.clear();
            }
            Message::CapsuleAngleChanged(angle) => {
                self.capsules[self.selected_capsule_index].angle = angle;
                self.init_world();
                self.cache.clear();
            }
        }

        Command::none()
    }

    fn view(&self) -> Element<Message> {
        let controls = view_controls(&self.capsules, self.selected_capsule_index, self.is_playing);

        let canvas = canvas(self as &Self)
            .width(Length::Fill)
            .height(Length::Fill);

        let debug_text = text(&*self.debug_text.borrow()).size(16);

        let content = column![canvas, debug_text, controls].height(Length::Fill);

        container(content)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    fn subscription(&self) -> Subscription<Message> {
        iced::time::every(std::time::Duration::from_millis(TIME_STEP)).map(|_| Message::Tick)
    }
}

impl<Message> canvas::Program<Message> for PhysicsSimulation {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &iced::Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        let selected_capsule = &self.capsules[self.selected_capsule_index];
        let selected_capsule_body = &self.rigid_body_set[selected_capsule.body_handle];
        let selected_capsule_position = selected_capsule_body.translation();
        let selected_capsule_velocity = selected_capsule_body.linvel();
        let selected_capsule_rotation = selected_capsule_body.rotation();

        *self.debug_text.borrow_mut() = format!(
            "Capsule position: ({:.2},{:.2}), velocity: ({:.2},{:.2}), angle: {:.2}",
            selected_capsule_position.x,
            selected_capsule_position.y,
            selected_capsule_velocity.x,
            selected_capsule_velocity.y,
            selected_capsule_rotation.angle()
        );

        let offset_factor = Vector::new(
            bounds.width / 2.0,
            bounds.height - (GROUND_HEIGHT * SCALE_FACTOR),
        );

        let physics_geometry: Geometry = self.cache.draw(renderer, bounds.size(), |frame| {
            // Clear the frame with the background color
            frame.fill_rectangle(Point::ORIGIN, bounds.size(), BACKGROUND_COLOR);

            // Draw the capsules
            for capsule in &self.capsules {
                let capsule_body = &self.rigid_body_set[capsule.body_handle];
                let capsule_position = capsule_body.translation();
                let capsule_rotation = capsule_body.rotation();

                // Scale and offset the capsule's position
                let scaled_capsule_position: Point = Point::new(
                    (-capsule_position.x * SCALE_FACTOR) + offset_factor.x,
                    offset_factor.y - (capsule_position.y * SCALE_FACTOR),
                );
                let scaled_capsule_radius: f32 = capsule.radius * SCALE_FACTOR;
                let scaled_capsule_half_length: f32 = capsule.length * SCALE_FACTOR / 2.0;

                // Draw the capsule
                let capsule_path = Path::line(
                    Point::new(-scaled_capsule_half_length, 0.0),
                    Point::new(scaled_capsule_half_length, 0.0),
                );
                let capsule_stroke_outline = Stroke {
                    style: Style::Solid(Color { r: 0.0, g: 0.0, b: 0.0, a: 1.0 }),
                    width: scaled_capsule_radius,
                    line_cap: LineCap::Round,
                    ..Stroke::default()
                };
                let capsule_stroke = Stroke {
                    style: Style::Solid(capsule.color),
                    width: scaled_capsule_radius - 1.0,
                    line_cap: LineCap::Round,
                    ..Stroke::default()
                };

                frame.with_save(|frame| {
                    frame.translate(iced::Vector::new(
                        scaled_capsule_position.x,
                        scaled_capsule_position.y,
                    ));
                    frame.rotate(capsule_rotation.angle());
                    frame.stroke(&capsule_path, capsule_stroke_outline);
                    frame.stroke(&capsule_path, capsule_stroke);
                });
            }

            // Draw the ground
            let ground_rect = Rectangle::new(
                Point::new(0.0, bounds.height - (GROUND_HEIGHT * SCALE_FACTOR)),
                Size::new(bounds.width, GROUND_HEIGHT * SCALE_FACTOR),
            );
            frame.fill_rectangle(ground_rect.position(), ground_rect.size(), GROUND_COLOR);
        });

        vec![physics_geometry]
    }
}

fn view_controls<'a>(
    capsules: &[Capsule],
    selected_capsule_index: usize,
    is_playing: bool,
) -> Element<'a, Message> {
    let mut playback_controls = Row::new().spacing(10);
    playback_controls = playback_controls
        .push(button(if is_playing { "Pause" } else { "Play" }).on_press(Message::TogglePlayback));
    if !is_playing {
        playback_controls =
            playback_controls.push(button("Step").on_press(Message::StepSimulation));
    }
    playback_controls = playback_controls.push(button("Reset").on_press(Message::ResetSimulation));

    let capsule_selector = capsules
        .iter()
        .enumerate()
        .fold(Row::new(), |row, (index, _capsule)| {
            row.push(
                button(text(format!("Capsule {}", index + 1)))
                    .style(if index == selected_capsule_index {
                        iced::theme::Button::Primary
                    } else {
                        iced::theme::Button::Secondary
                    })
                    .on_press(Message::CapsuleSelected(index)),
            )
        });

    let selected_capsule = &capsules[selected_capsule_index];

    let length_slider = column![
        text("Capsule Length"),
        slider(
            1.0..=10.0,
            selected_capsule.length,
            Message::CapsuleLengthChanged
        )
        .step(0.1)
    ];
    let radius_slider = column![
        text("Capsule radius"),
        slider(
            0.1..=5.0,
            selected_capsule.radius,
            Message::CapsuleRadiusChanged
        )
        .step(0.1)
    ];
    let angle_slider = column![
        text("Capsule angle"),
        slider(
            -std::f32::consts::PI..=std::f32::consts::PI,
            selected_capsule.angle,
            Message::CapsuleAngleChanged,
        )
        .step(0.1)
    ];

    column![
        playback_controls,
        capsule_selector,
        length_slider,
        radius_slider,
        angle_slider,
    ]
    .padding(10)
    .spacing(20)
    .align_items(Alignment::Center)
    .into()
}
