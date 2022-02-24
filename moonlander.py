#
# MOON LANDER (ASTERÓIDES)
#
from easymunk import Vec2d, Arbiter, Body, Vec2d, ShapeFilter, pyxel as phys
import pyxel
import random

FPS = 30
WIDTH, HEIGHT = 256, 196
SCREEN = Vec2d(WIDTH, HEIGHT)


class Particles:
    def __init__(self, space):
        self.particles = []
        self.space = space

    def draw(self, camera=pyxel):
        for p in self.particles:
            x, y = p.position
            if random.random() < 0.15:
                camera.rect(x, y, 2, 2, self.get_color(p.duration))
            else:
                camera.pset(x, y, self.get_color(p.duration))

    def update(self):
        for p in self.particles.copy():
            p.velocity = p.velocity.rotated(random.uniform(-5, 5)) 
            p.duration -= 1
            if p.duration <= 0:
                self.particles.remove(p)
                p.detach()

    def emmit(self, position, velocity):        
        p = self.space.create_circle(
            radius=1,
            mass=0.1,
            moment=float("inf"),
            position=position,
            velocity=velocity,
            filter=ShapeFilter(group=1),
        )
        p.duration = 105 - random.expovariate(1 / 10)
        p.velocity_func = self.update_velocity
        self.particles.append(p)

    def update_velocity(self, body, gravity, damping, dt):
        body.update_velocity(body, -gravity / 2, 0.99, dt)

    def get_color(self, t):
        if t > 95:
            return pyxel.COLOR_WHITE
        elif t > 80:
            return pyxel.COLOR_CYAN
        elif t > 65:
            return pyxel.COLOR_RED
        elif t > 40:
            return pyxel.COLOR_PURPLE
        elif t > 25:
            return pyxel.COLOR_BROWN
        else:
            return pyxel.COLOR_GRAY


class FireParticles:  # Classe responsável pelos efeitos de explosão
    def __init__(self, space):
        self.particles = []
        self.space = space

    def draw(self, camera=pyxel):
        for p in self.particles:
            x, y = p.position
            if random.random() < 0.45:
                camera.rect(x, y, 2, 2, self.get_color(p.duration, p.type_color))
            else:
                camera.pset(x, y, self.get_color(p.duration, p.type_color))

    def update(self):
        for p in self.particles.copy():
            p.velocity = p.velocity.rotated(random.uniform(-0.5, 0.5)) 
            p.duration -= 0.65
            if p.duration <= 0:
                self.particles.remove(p)
                p.detach()

    def emmit(self, position, velocity, color_type):        
        p = self.space.create_circle(
            radius=5,
            mass=0.1,
            moment=float("inf"),
            position=position,
            velocity=velocity,
            filter=ShapeFilter(group=1),
        )
        p.duration = 105 - random.expovariate(1 / 10)
        p.velocity_func = self.update_velocity
        p.type_color = color_type
        self.particles.append(p)

    def update_velocity(self, body, gravity, damping, dt):
        body.update_velocity(body, -gravity / 2, 0.99, dt)

    def get_color(self, t, color_type):
        if color_type == 1: # Fumaça
            if t > 80:
                return pyxel.COLOR_ORANGE
            else:
                return pyxel.COLOR_GRAY
        elif color_type == 2: # Explosão
            if t > 95:
                return pyxel.COLOR_YELLOW
            elif t > 45:
                return pyxel.COLOR_ORANGE
            else:
                return pyxel.COLOR_GRAY
        elif color_type == 3: # Fogo
            if t > 92:
                return pyxel.COLOR_RED
            elif t > 75:
                return pyxel.COLOR_YELLOW
            elif t > 30:
                return pyxel.COLOR_ORANGE
            elif t > 25:
                return pyxel.COLOR_BROWN
            else:
                return pyxel.COLOR_GRAY


class Game:
    PLAYER_SHAPE = [(0, 6), (-3, -3), (+3, -3)]
    BASE_SHAPE = (25, 5)
    PLAYER_SPEED = 90
    PLAYER_COLOR = pyxel.COLOR_WHITE
    BASE_COLOR = pyxel.COLOR_PINK
    ASTRD_COLOR = pyxel.COLOR_BROWN
    GRAVITY = Vec2d(0, -25)
    THRUST = -3 * GRAVITY
    ANGULAR_VELOCITY = 180
    FLOOR_STEP = 30
    FLOOR_DY = 15
    FLOOR_N = 20
    GROUND_N = 30
    GROUND_STEP = 45
    PLAYER_COL_TYPE = 1
    BASE_COL_TYPE = 2
    FLOOR_COL_TYPE = 3
    ASTRD_COL_TYPE = 4
    MAX_IMPULSE = 40
    PROB_CREATE_ASTEROID = 1 / 30

    def __init__(self):
        self.space = space = phys.space(
            gravity=self.GRAVITY,
            camera=phys.Camera(flip_y=True),
            friction=1,
        )

        # Variáveis da lógica do jogo
        self.landed = False
        self.victory = False
        self.youlose_time = 45
        self.old_ply_pos = self.new_plyr_pos = Vec2d(0, 0)
        self.win_time = 60
        self.astd_colisao = None
        self.condition = True

        # Varáveis de partículas
        self.particles = Particles(space)
        self.fireparticles = FireParticles(space)

        self.asteroids = []

        # Cria jogador
        self.player = space.create_poly(
            self.PLAYER_SHAPE,
            mass=1,
            moment=2,
            position=SCREEN / 2,
            friction=1.0,
            collision_type=self.PLAYER_COL_TYPE,
            filter=ShapeFilter(group=1),
            color= self.PLAYER_COLOR
        )

        # Cria base
        dx = random.uniform(-WIDTH, WIDTH)
        self.base = space.create_box(
            self.BASE_SHAPE,
            position=self.player.position + (dx*1.2, 0.55 * HEIGHT),
            friction=1.0,
            collision_type=self.BASE_COL_TYPE,
            body_type=Body.STATIC,
        )
        self.player.position = self.base.position + Vec2d(random.uniform(-100, 100), random.uniform(100, 200))

        # Cria chão
        shape = list(self.base.shapes)[0]
        bb = shape.cache_bb()
        self.make_floor(bb.right, bb.bottom, self.FLOOR_STEP, self.FLOOR_DY)
        self.make_floor(bb.left, bb.bottom, -self.FLOOR_STEP, self.FLOOR_DY)

        # Cria linha reta abaixo da base
        bb = shape.cache_bb()
        self.make_ground(bb.bottom, bb.bottom-300, self.GROUND_STEP)
        self.make_ground(bb.bottom, bb.bottom-300, -self.GROUND_STEP)

        # Escuta colisões entre base/chão/asteróides e jogador
        space.collision_handler(
            self.PLAYER_COL_TYPE, self.BASE_COL_TYPE, post_solve=self.on_land
        )
        self.space.collision_handler(
            self.PLAYER_COL_TYPE, self.FLOOR_COL_TYPE, begin=self.on_collision
        )
        self.space.collision_handler(
            self.PLAYER_COL_TYPE, self.ASTRD_COL_TYPE, post_solve=self.asteroid_collision
        )

    def on_collision(self, arb: Arbiter):
        self.landed = True
        self.victory = False
        return True

    def on_land(self, arb: Arbiter):
        if not self.landed:
            self.victory = arb.total_impulse.length < self.MAX_IMPULSE
        self.landed = True

    def make_floor(self, x, y, step, dy):
        body = self.space.static_body

        a = Vec2d(x, y)
        for _ in range(self.FLOOR_N):
            b = a + (step, random.uniform(-dy, dy))
            body.create_segment(a, b, 1, collision_type=self.FLOOR_COL_TYPE)
            a = b
            body.friction = 0.7
    
    def make_ground(self, x, y, step):
        body = self.space.static_body

        a = Vec2d(x, y)
        for _ in range(self.GROUND_N):
            b = a + (step, 0)
            body.create_segment(a, b, 1)
            a = b
            body.friction = 1.0

    def update(self):
        if not self.landed:
            if pyxel.btn(pyxel.KEY_LEFT):
                self.player.angular_velocity = +self.ANGULAR_VELOCITY
            elif pyxel.btn(pyxel.KEY_RIGHT):
                self.player.angular_velocity = -self.ANGULAR_VELOCITY
            else:
                self.player.angular_velocity = 0.0

            if pyxel.btn(pyxel.KEY_UP):
                self.player.apply_force_at_local_point(4 * self.THRUST)                
                
                for _ in range(2):
                    self.particles.emmit(
                        position=self.player.local_to_world((random.uniform(-2, 2), -3)),
                        velocity= -random.uniform(50, 90) * self.player.rotation_vector.perpendicular(),
                    )
                    
        else: # Processo de explosão
            if not self.victory:

                # Compara a posição anterior da nave com a atual
                self.new_plyr_pos = self.player.position - self.old_ply_pos
                self.old_ply_pos = self.player.position

                # Caso a nave tenha encostado em um asteróide
                if self.astd_colisao:
                    if self.player.color != pyxel.COLOR_GRAY:
                        for _ in range(200): # Explosão maior que acontece apenas uma vez
                            self.fireparticles.emmit(
                                position= self.player.local_to_world((random.uniform(-2, 2), -1)),
                                velocity= 70 * self.player.rotation_vector.perpendicular().rotated(random.uniform(0, 360)),
                                color_type= 2
                            )
                    
                    self.player.color = pyxel.COLOR_GRAY

                    # A nave começa a emitir fumaça
                    self.fireparticles.emmit(
                        position= self.player.local_to_world((random.uniform(-0.5, 0.5), 6)),
                        velocity= random.uniform(10, 30) * self.player.rotation_vector.perpendicular(),
                        color_type= 1
                    )
                
                elif self.astd_colisao is None:
                    # A nave começa a emitir fumaça
                    self.fireparticles.emmit(
                        position= self.player.local_to_world((random.uniform(-0.5, 0.5), 6)),
                        velocity= random.uniform(10, 30) * self.player.rotation_vector.perpendicular(),
                        color_type= 1
                    )

                # Espera um certo tempo depois que a nave parou por completo

                # 1° if para caso a nave não tenha colidido com um asteróide
                if self.new_plyr_pos == (0, 0) and self.player.color != pyxel.COLOR_GRAY:
                    self.youlose_time -= 1
                
                    # Explode depois que o tempo acaba
                    if self.youlose_time <= 0:
                        if self.player.color != pyxel.COLOR_GRAY:
                            for _ in range(200): # Explosão maior que acontece apenas uma vez
                                self.fireparticles.emmit(
                                    position= self.player.local_to_world((random.uniform(-2, 2), -1)),
                                    velocity= 70 * self.player.rotation_vector.perpendicular().rotated(random.uniform(0, 360)),
                                    color_type= 2
                                )
                            self.condition = False
                        self.player.color = pyxel.COLOR_GRAY
                        
                        self.fireparticles.emmit(
                            position= self.player.local_to_world((random.uniform(-3, 3), -2)),
                            velocity= 5 * Vec2d(random.uniform(-3, 3), random.uniform(-1, 4)),
                            color_type= 3
                        )
                
                # 2° if para caso a nave tenha colidido com um asteróide
                elif self.new_plyr_pos == (0, 0) and self.player.color == pyxel.COLOR_GRAY:
                    self.astd_colisao = False
                    if self.condition:
                        for _ in range(200): # Explosão maior que acontece apenas uma vez
                            self.fireparticles.emmit(
                                position= self.player.local_to_world((random.uniform(-2, 2), -1)),
                                velocity= 20 * self.player.rotation_vector.perpendicular().rotated(random.uniform(0, 360)),
                                color_type= 2
                            )
                        self.condition = False
                    self.fireparticles.emmit(
                        position= self.player.local_to_world((random.uniform(-3, 3), -2)),
                        velocity= 5 * Vec2d(random.uniform(-3, 3), random.uniform(-1, 4)),
                        color_type= 3
                    )
            
        
        self.update_asteroids()

        dt = 1 / FPS
        self.particles.update()
        self.fireparticles.update()
        self.space.step(dt, sub_steps=4)
        self.space.camera.follow(self.player.position)

    def update_asteroids(self):
        if random.random() < self.PROB_CREATE_ASTEROID and not self.landed:
            self.create_asteroid()
             
        new_asteroids = []
        for asteroid in self.asteroids:
            asteroid.life -= 1
            if asteroid.life <= 0 or self.landed:
                asteroid.detach()
            else:
                new_asteroids.append(asteroid)
        self.asteroids = new_asteroids

    def asteroid_collision(self, arb: Arbiter):
        self.landed = True
        self.victory = False
        self.astd_colisao = True
        return True

    def create_asteroid(self):
        pos = self.player.position + Vec2d(random.uniform(-20, 20), random.uniform(45, 60))
        size = 10
        points = []
        
        for _ in range(10):
            dx = random.uniform(-size, size)
            dy = random.uniform(-size, size)
            points.append(pos + (dx, dy))

        asteroid = self.space.create_poly(points, collision_type=self.ASTRD_COL_TYPE, color= self.ASTRD_COLOR)
        asteroid.life = 126
        self.asteroids.append(asteroid)

    def draw(self):
        pyxel.cls(0)

        a, b = 15, 15
        d= (self.base.position - self.player.position).normalized()
        start = self.player.position + a *d
        # end = start + b*d

        pontatri = start + Vec2d(0, 5).rotated(d.angle-90)
        vertice1 = start + Vec2d(-3.5, 0).rotated(d.angle-90)
        vertice2 = start + Vec2d(+3.5, 0).rotated(d.angle-90)
        
        camera = self.space.camera

        if not self.landed:
            camera.trib(*pontatri, *vertice1, *vertice2, pyxel.COLOR_YELLOW)
        
        # camera.line(*start, *end, pyxel.COLOR_CYAN)
        camera.draw(self.player)
        camera.draw(self.space.static_body)
        self.particles.draw(camera)
        self.fireparticles.draw(camera)
        camera.draw(self.base)

        for asteroid in self.asteroids:
            camera.drawb(asteroid)

        if self.landed:
            a = 1
            msg = "PARABENS!" if self.victory else "PERDEU :("
            x = WIDTH / 2 - len(msg) * pyxel.FONT_WIDTH / 2
            if msg == "PARABENS!" and a == 1:
                self.win_time -= 1
                if self.win_time <= 0:
                    a = 2
                    pyxel.text(x, HEIGHT // 2 - 20, msg, pyxel.COLOR_RED)
            else: pyxel.text(x, HEIGHT // 2 - 40, msg, pyxel.COLOR_RED)

game = Game()
pyxel.init(WIDTH, HEIGHT, fps=FPS)
pyxel.mouse(True)
pyxel.run(game.update, game.draw)
