import math
from matplotlib import pyplot as plt
import pygame
from sys import exit


class Body:
    """Class for a projectile with mass, radius, position, velocity, angle, angular velocity, drag, target, xy_force, and angle_force"""

    def __init__(self, mass = 1, radius = .05, position = (0, 0, 122), velocity = (0, 0, 0), angle = 0, angle_vel = 0, drag = (0,0,0), target = (0,0), xy_force = (0,0), angle_force = 0):
        # constants
        self.mass = mass
        self.radius = radius
        self.drag = drag
        self.target = target
        self.target_vel = (0,0)
        self.target_angle_vel = 0
        self.controller = Controller(self)

        # state variables
        self.position = list(position)
        self.velocity = list(velocity)
        self.angle = angle
        self.angle_vel = angle_vel

        # error variables
        self.pos_error = [0,0]
        self.vel_error = [0,0]
        self.ang_error = 0
        self.ang_vel_error = 0
        

        # force variables
        self.xy_force = list(xy_force)
        self.angle_force = angle_force
    
    
    def step(self, net_accel, time_step = 0.01):
        """Integrates xyz/angular accelerations to corresponding velocity and position of projectile"""
        
        # integrates to find new position and velocity
        for i in range(3):
            self.velocity[i] += net_accel[i] * time_step
            self.position[i] += self.velocity[i] * time_step

        # integrates to find new angle and angular velocity
        self.angle_vel += net_accel[3] * time_step
        self.angle += self.angle_vel * time_step
        self.angle = self.angle % 360
        if self.angle < 0:
            self.angle += 360
        
    """updates position, velocity, and angle errors"""
    def update_pos_error(self):
        self.pos_error = [curr-target for curr, target in zip(self.position, self.target)]
    
    def update_vel_error(self):
        self.vel_error = [velocity-target for velocity, target in zip(self.velocity, self.target_vel)]
        
    def update_angle_error(self):
        self.ang_error =  self.angle
        if self.ang_error > 180:
            self.ang_error -= 360

    def update_ang_vel_error(self):
        self.ang_vel_error = self.angle_vel - self.target_angle_vel
    
    """formats terminal output"""
    def pos_report(self):
        return f"pos({self.position[0]:0.2f}, {self.position[1]:0.2f}, {self.position[2]:0.2f})"

    def vel_report(self):
        return f"vel({self.velocity[0]:0.2f}, {self.velocity[1]:0.2f}, {self.velocity[2]:0.2f})"
    
    def ang_report(self):
        return f"angle({self.angle:0.2f})"
    
    def ang_vel_report(self):
        return f"ang_vel({self.ang_vel:0.2f})"
    
    def pos_error_report(self):
        return f"pos_error({self.pos_error[0]:0.2f}, {self.pos_error[1]:0.2f})"

    def vel_error_report(self):
        return f"vel_error({self.vel_error:0.2f})"

    def ang_error_report(self):
        return f"ang_error({self.ang_error:0.2f})"

class Controller:
    """Class for a PID controller with a body, force limit, and PID constants"""
    def __init__(self, body:Body):
        self.body = body
        self.force_lim = 50
        self.angle_force_lim = 10
        
        # xy PID constants
        self.p = 15
        self.i = .15
        self.d = 5

        # angle PID constants
        self.ang_p = 2
        self.ang_i = .001
        self.ang_d = .2


    def correct_xy(self, integral_x, integral_y):
        """Calculates xy force to apply to body"""

        # x axis
        force_x = self.body.pos_error[0] * -self.p
        force_x += self.body.velocity[0] * -self.d
        force_x += integral_x * -self.i

        # y axis
        force_y = self.body.pos_error[1] * -self.p
        force_y += self.body.velocity[1] * -self.d
        force_y += integral_y * -self.i

        # limit force
        if force_x > self.force_lim:
            force_x = self.force_lim
        elif force_x < -self.force_lim:
            force_x = -self.force_lim

        if force_y > self.force_lim:
            force_y = self.force_lim
        elif force_y < -self.force_lim:
            force_y = -self.force_lim


        self.body.xy_force = [force_x, force_y]


    def correct_angle(self, integral_ang):
        """Calculates angular force to apply to body"""
        
        # angular force
        ang_force = self.body.ang_error * -self.ang_p
        ang_force += self.body.ang_vel_error * -self.ang_d
        ang_force += integral_ang * -self.ang_i

        # limit force
        if ang_force > self.angle_force_lim:
            ang_force = self.angle_force_lim
        elif ang_force < -self.angle_force_lim:
            ang_force = -self.angle_force_lim


        self.body.angle_force = ang_force
        
class Simulator:
    """class that simulates a projectile in 3D space"""

    def __init__(self, body:Body, wind_accel = (0,0), time_step = 0.01, debug = False):
        self.debug = debug

        # constants
        self.body = body
        self.time_step = time_step
        self.step_count = 0
        self.wind = wind_accel

        # lists to store data for graphing
        self.time = []

        self.x_pos = []
        self.y_pos = []
        self.z_pos = []

        self.x_vel = []
        self.y_vel = []
        self.z_vel = []
        
        self.x_accel = []
        self.y_accel = []
        self.z_accel = []

        self.angle = []
        self.angle_vel = []
        self.angle_accel = []


    def run(self):
        """Runs simulation until body reaches ground"""

        integral_x = 0
        integral_y = 0
        integral_ang = 0

        while self.body.position[2] > 0:
            body_accel = net_accel(self.body, self)
            self.body.step(body_accel)
            self.step_count +=1

            # update errors
            self.body.update_pos_error()
            self.body.update_vel_error()
            self.body.update_angle_error()
            self.body.update_ang_vel_error()
            
            # update controller every 5 steps
            if self.step_count % 1 == 0:
                self.body.controller.correct_angle(integral_ang)
                self.body.controller.correct_xy(integral_x, integral_y)


            # Sets integrals for PID loops
            if self.body.pos_error[0] < 1:
                integral_x += self.body.pos_error[0]
            else:
                integral_x = 0

            if self.body.pos_error[1] < 1:
                integral_y += self.body.pos_error[1]
            else:
                integral_y = 0

            if self.body.ang_error < 1:
                integral_ang += self.body.ang_error
            else:
                integral_ang = 0


            # append data to lists for plotting
            self.time.append(self.time_step*self.step_count)

            self.x_pos.append(self.body.position[0])
            self.y_pos.append(self.body.position[1])
            self.z_pos.append(self.body.position[2])
            
            self.x_vel.append(self.body.velocity[0])
            self.y_vel.append(self.body.velocity[1])
            self.z_vel.append(self.body.velocity[2])

            self.x_accel.append(body_accel[0])
            self.y_accel.append(body_accel[1])
            self.z_accel.append(body_accel[2])

            self.angle.append(self.body.angle)
            self.angle_vel.append(self.body.angle_vel)
            self.angle_accel.append(body_accel[3])

            if self.debug:
                print(
                    f"time: {self.step_count*self.time_step:0.3f}, {self.body.pos_report()}, {self.body.vel_report()}, {self.body.ang_report()}, {self.body.pos_error_report()}, {self.body.ang_error_report()}"
                )
    
    def draw_frame(self, screen, sim_is_running):
        """Draws frame of simulation"""
        return False

def net_accel(body:Body, sim:Simulator):
    """Calculates net acceleration of projectile"""

    # define constants
    gravity = -9.81
    body_moi = 1/2 * body.mass * (body.radius ** 2)
    convert = 180/math.pi


    x_accel = body.drag[0] * ((body.velocity[0]) ** 2) + sim.wind[0] + body.xy_force[0] *  math.cos(body.angle/convert) + body.xy_force[1] *  math.sin(body.angle/convert)
    y_accel = body.drag[1] * ((body.velocity[1]) ** 2) + sim.wind[1] + body.xy_force[1] *  math.cos(body.angle/convert) + body.xy_force[0] *  math.sin(body.angle/convert)
    z_accel = gravity * body.mass + body.drag[2] * ((body.velocity[2]) ** 2)
    torque = body.angle_force


    return x_accel/body.mass, y_accel/body.mass, z_accel/body.mass, torque/body_moi

def run_skydiver_simulation(animate_sim):
    """builds projectile and simulator and runs simulation"""
    if animate_sim: animate()
    body = Body(mass=1, position=[0, 0, 122], velocity=[0, 0, 0], drag=(0, 0, 0), target=(-7,-15), angle = 181)
    sim = Simulator(body, wind_accel = (0,0), debug=True)
    sim.run()
    graph(sim, body)

def graph(sim:Simulator, body:Body):
    """graphs data from simulation"""

    # create a figure with three subplots
    fig, axs = plt.subplots(4, 1, figsize=(8, 12), sharex=True)
    axs2 = plt.subplots(1, 1, figsize=(8, 12), sharex=True)

    # plot position data
    axs[0].plot(sim.time, sim.x_pos, label='x')
    axs[0].plot(sim.time, sim.y_pos, label='y')
    axs[0].axhline(body.target[0], color='r', linestyle='--', linewidth=1)
    axs[0].axhline(body.target[1], color='r', linestyle='--',linewidth=1)
    axs[0].set_ylabel('Position (m)')
    axs[0].legend()

    # plot velocity data
    axs[1].plot(sim.time, sim.x_vel, label='x')
    axs[1].plot(sim.time, sim.y_vel, label='y')
    axs[1].plot(sim.time, sim.z_vel, label='z')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].legend()

    # plot acceleration data(z not plotted)
    axs[2].plot(sim.time, sim.x_accel, label='x')
    axs[2].plot(sim.time, sim.y_accel, label='y')

    axs[2].set_ylabel('Acceleration (m/s^2)')
    axs[2].set_xlabel('Time (s)')
    axs[2].legend()
    
    # angle error
    axs[3].plot(sim.time, sim.angle)
    axs[3].axhline(0, color='r', linestyle='--', linewidth=1)
    axs[3].set_xlabel('Time (s)')
    axs[3].set_ylabel('Angle (deg)')


    # add a title to the figure
    fig.suptitle('Position, Velocity, Angle, and Acceleration vs. Time')
    
    # 3d position plot
    axs2 = plt.axes(projection='3d')
    axs2.plot(sim.x_pos, sim.y_pos, sim.z_pos, 'red')

    plt.show()

def animate():
    """animates data from simulation"""
    pygame.init()
    screen= pygame.display.set_mode((1000,700))
    pygame.display.set_caption("Simulator")
    clock = pygame.time.Clock()
    sim_font = pygame.font.SysFont("comicsans", 30)
    data_font = pygame.font.SysFont("comicsans", 25)


    sim_state = 0

    # --------------------------------------------main screen[0]-----------------------------------------------
    title_0 = sim_font.render("This is a Projectile Simulator", True, ("black"))
    title_0_rect = title_0.get_rect(midtop=(500,50))
    
    next_0 = sim_font.render("Press to create a simulation", True, ("black"))
    next_0_rect = next_0.get_rect(midtop=(500,500))
    

    # -------------------------------------------data entered screen[1]-----------------------------------------
    text_color = "Black"
    left_margin = 50

    # title
    title_1 = sim_font.render("Enter flight parameters", 1, text_color)
    title_1_rect = title_1.get_rect(midtop=(500,20))
    
    # continue button
    next_1 = data_font.render("Simulate", 1, text_color)
    next_1_rect = next_1.get_rect(bottomright=(950,650))

    # starting position
    pos_label = data_font.render("Starting Position:", 1, text_color)
    pos_label_rect = pos_label.get_rect(topleft=(left_margin,100))
    
    x_pos_label = data_font.render("x(m):", 1, text_color)
    x_pos_label_rect = x_pos_label.get_rect(topleft=(pos_label_rect.right+20,100))
    input_pos_x_rect = pygame.Rect(x_pos_label_rect.right+10, 100, 75, 40)
    
    y_pos_label = data_font.render("y(m):", 1, text_color)
    y_pos_label_rect = y_pos_label.get_rect(topleft=(input_pos_x_rect.right+25,100))
    input_pos_y_rect = pygame.Rect(y_pos_label_rect.right+10, 100, 75, 40)
    
    z_pos_label = data_font.render("z(m):", 1,text_color)
    z_pos_label_rect = z_pos_label.get_rect(topleft=(input_pos_y_rect.right+25,100))
    input_pos_z_rect = pygame.Rect(z_pos_label_rect.right+10, 100, 75, 40)

    pos = Data_Cat(screen=screen, 
                   font=data_font, 
                   label=pos_label, 
                   label_rect = pos_label_rect, 
                   x_label = x_pos_label, 
                   x_label_rect = x_pos_label_rect, 
                   y_label = y_pos_label, 
                   y_label_rect = y_pos_label_rect, 
                   z_label = z_pos_label, 
                   z_label_rect = z_pos_label_rect, 
                   x_input_rect = input_pos_x_rect, 
                   y_input_rect = input_pos_y_rect, 
                   z_input_rect = input_pos_z_rect,
                   type=0)

    # starting velocity
    vel_label = data_font.render("Starting Velocity:", 1, text_color)
    vel_label_rect = vel_label.get_rect(topleft=(left_margin,165))

    x_vel_label = data_font.render("x(m/s):", 1, text_color)
    x_vel_label_rect = x_vel_label.get_rect(topleft=(vel_label_rect.right+20,165))
    input_vel_x_rect = pygame.Rect(x_vel_label_rect.right+10, 165, 100, 40)
    
    y_vel_label = data_font.render("y(m/s):", 1, text_color)
    y_vel_label_rect = y_vel_label.get_rect(topleft=(input_vel_x_rect.right+10,165))
    input_vel_y_rect = pygame.Rect(y_vel_label_rect.right+10, 165, 100, 40)
    
    z_vel_label = data_font.render("z(m/s):", 1, text_color)
    z_vel_label_rect = z_vel_label.get_rect(topleft=(input_vel_y_rect.right+10,165))
    input_vel_z_rect = pygame.Rect(z_vel_label_rect.right+10, 165, 100, 40)
    
    vel = Data_Cat(screen=screen, 
                   font=data_font, 
                   label=vel_label, 
                   label_rect = vel_label_rect, 
                   x_label = x_vel_label, 
                   x_label_rect = x_vel_label_rect, 
                   y_label = y_vel_label, 
                   y_label_rect = y_vel_label_rect, 
                   z_label = z_vel_label, 
                   z_label_rect = z_vel_label_rect, 
                   x_input_rect = input_vel_x_rect, 
                   y_input_rect = input_vel_y_rect, 
                   z_input_rect = input_vel_z_rect,
                   type=0)
    
    # starting angle
    angle_label = data_font.render("Starting Angle(degrees):", 1, text_color)
    angle_label_rect = angle_label.get_rect(topleft=(50,225))
    input_angle_rect = pygame.Rect(angle_label_rect.right+10, 225, 200, 40)

    angle = Data_Cat(screen= screen, 
                     font=data_font, 
                     label=angle_label, 
                     label_rect = angle_label_rect, 
                     x_input_rect = input_angle_rect,
                     type=1)

    # starting mass
    mass_label = data_font.render("Starting Mass(kg):", 1, text_color)
    mass_label_rect = mass_label.get_rect(topleft=(50,285))
    input_mass_rect = pygame.Rect(mass_label_rect.right+10, 285, 200, 40)

    mass = Data_Cat(screen=screen, 
                    font=data_font, 
                    label=mass_label, label_rect=mass_label_rect, 
                    x_input_rect=input_mass_rect,
                    type=1)

    # target
    target_label = data_font.render("Target:", 1, text_color)
    target_label_rect = target_label.get_rect(topleft=(left_margin,550))
    
    x_target_label = data_font.render("x(m):", 1, text_color)
    x_target_label_rect = x_target_label.get_rect(topleft=(left_margin,target_label_rect.bottom+10))
    input_target_x_rect = pygame.Rect(x_target_label_rect.right + 10, target_label_rect.bottom+10, 50, 40)
    
    y_target_label = data_font.render("y(m):", 1, text_color)
    y_target_label_rect = y_target_label.get_rect(topleft=(input_target_x_rect.right+50,target_label_rect.bottom+10))
    input_target_y_rect = pygame.Rect(y_target_label_rect.right + 10, target_label_rect.bottom+10, 50, 40)
    

    target = Data_Cat(screen=screen, 
                      font=data_font, 
                      label=target_label, 
                      label_rect = target_label_rect, 
                      x_label = x_target_label, 
                      x_label_rect = x_target_label_rect, 
                      y_label = y_target_label, 
                      y_label_rect = y_target_label_rect, 
                      x_input_rect = input_target_x_rect, 
                      y_input_rect = input_target_y_rect,
                      type = 2)

    # starting wind
    wind = sim_font.render("Wind Type:", 1, text_color)
    wind_rect = wind.get_rect(topleft=(50,500))

    constant_wind = sim_font.render("Constant Wind", 1, text_color)
    constant_wind_rect = constant_wind.get_rect(topleft=(50,550))
    wind_x_rect = pygame.Rect(50, 600, 100, 50)
    wind_y_rect = pygame.Rect(200, 600, 100, 50)

    rand_wind_button = sim_font.render("Random Wind", 1, text_color)
    rand_wind_rect = rand_wind_button.get_rect(topleft=(50,550))

    no_wind_button = sim_font.render("No Wind", 1, text_color)
    no_wind_rect = no_wind_button.get_rect(topleft=(250,550))

    # list of data categories
    data_list = [pos,vel,angle,mass,target]
    

    # --------------------------------------Simulation screen[2]----------------------------------
    next_2 = sim_font.render("Press to Continue", 1, text_color)
    next_2_rect = next_2.get_rect(topleft=(50,50))
    sim_is_running = True


    # ----------------------------------------End screen[3]-----------------------------------------
    title_3 = sim_font.render("Simulation Complete", 1, text_color)
    title_3_rect = title_3.get_rect(midtop=(500,50))

    next_3 = sim_font.render("Press Space to start new simulation", 1, text_color)
    next_3_rect = next_3.get_rect(midtop=(500,600))
    
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                # If the user clicked on the input_box rect.
                for data in data_list:
                    data.update_active(event.pos)
            if event.type == pygame.KEYDOWN:
                # puts input into boxes
                for data in data_list:
                    data.update_input(event)

        # intro screen
        if sim_state == 0:
            screen.fill("#c0e8ec")
            screen.blit(title_0,title_0_rect)
            pygame.draw.rect(screen,"Green", next_0_rect)
            pygame.draw.rect(screen,"Green", next_0_rect,6)
            screen.blit(next_0,next_0_rect)

            # changes color of button on hover
            if next_0_rect.collidepoint(pygame.mouse.get_pos()):
                pygame.draw.rect(screen,"Red", next_0_rect)
                pygame.draw.rect(screen,"Red", next_0_rect,6)
                screen.blit(next_0,next_0_rect)

            # changes screen on click
            if pygame.mouse.get_pressed()[0] and next_0_rect.collidepoint(pygame.mouse.get_pos()):
                sim_state += 1
                sim_state = sim_state % 4

        # Data entered screen
        elif sim_state == 1:
            screen.fill("#c0e8ec")

            # draws title and continue button
            screen.blit(title_1,title_1_rect)
            pygame.draw.rect(screen,"Green", next_1_rect)
            pygame.draw.rect(screen,"Green", next_1_rect,6)
            screen.blit(next_1, next_1_rect)

            # draws data categories
            for data in data_list:
                data.draw()

           
            # changes color of button on hover
            if next_1_rect.collidepoint(pygame.mouse.get_pos()):
                pygame.draw.rect(screen,"Red", next_1_rect)
                pygame.draw.rect(screen,"Red", next_1_rect,6)
                screen.blit(next_1,next_1_rect)

            # changes screen on click
            if pygame.mouse.get_pressed()[0] and next_1_rect.collidepoint(pygame.mouse.get_pos()):
                sim_state = (sim_state + 1) % 4
                

                # get data from input boxes
                pos_data = [int(data_list[0].x_input), int(data_list[0].y_input), int(data_list[0].z_input)]
                vel_data =[int(data_list[1].x_input), int(data_list[1].y_input), int(data_list[1].z_input)]
                angle_data = int(data_list[2].x_input)
                mass_data = int(data_list[3].x_input)
                target_data = [int(data_list[4].x_input), int(data_list[4].y_input)]
                wind_data = (0,0)
            
                body = Body(mass=mass_data, position=pos_data, velocity=vel_data, drag=(0, 0, 0), target=target_data, angle = angle_data)
                sim = Simulator(body, wind_accel = wind_data)
                sim.run()

        # Simulation screen  
        elif sim_state == 2:
            if sim_is_running:
                sim_is_running = sim.draw_frame(screen=screen, sim_is_running=sim_is_running)
            else:
                screen.fill("#c0e8ec")
                
                pygame.draw.rect(screen,"Green", next_2_rect)
                pygame.draw.rect(screen,"Green", next_2_rect,6)
                screen.blit(next_2, next_2_rect)

                # changes color of button on hover
                if next_2_rect.collidepoint(pygame.mouse.get_pos()):
                    pygame.draw.rect(screen,"Red", next_2_rect)
                    pygame.draw.rect(screen,"Red", next_2_rect,6)
                    screen.blit(next_2,next_2_rect)

                # changes screen on click
                if pygame.mouse.get_pressed()[0] and next_2_rect.collidepoint(pygame.mouse.get_pos()):
                    sim_state = (sim_state + 1) % 4

                    # get final error
                    error_msg = sim_font.render(f"{body.pos_error_report()}", 1, (0,0,0))
                    error_rect = error_msg.get_rect(midtop=(500,100))    
        # end screen
        else:
            screen.fill("#c0e8ec")
            screen.blit(title_3, title_3_rect)
            screen.blit(error_msg, error_rect)
            pygame.draw.rect(screen,"Green", next_3_rect)
            pygame.draw.rect(screen,"Green", next_3_rect,6)
            screen.blit(next_3, next_3_rect)

            # changes color of button on hover
            if next_3_rect.collidepoint(pygame.mouse.get_pos()):
                pygame.draw.rect(screen,"Red", next_3_rect)
                pygame.draw.rect(screen,"Red", next_3_rect,6)
                screen.blit(next_3,next_3_rect)

            # changes screen on click
            if pygame.mouse.get_pressed()[0] and next_3_rect.collidepoint(pygame.mouse.get_pos()):
                sim_state += 1
                sim_state = sim_state % 4

        pygame.display.update()
        clock.tick(60)
class Data_Cat:
    def __init__(self,screen, font, label, label_rect, x_input_rect, x_label = None, x_label_rect = None, y_label = None, y_label_rect= None, y_input_rect= None, z_label = None, z_label_rect = None, z_input_rect = None, type = None):
        self.screen = screen
        self.font = font
        self.label = label
        self.label_rect = label_rect


        self.x_input_rect = x_input_rect
        self.y_input_rect = y_input_rect
        self.z_input_rect = z_input_rect
        self.x_input = ""
        self.y_input = ""
        self.z_input = ""

        self.x_label = x_label
        self.y_label = y_label
        self.z_label = z_label
        self.x_label_rect = x_label_rect
        self.y_label_rect = y_label_rect
        self.z_label_rect = z_label_rect

        self.x_active = False
        self.y_active = False
        self.z_active = False
        self.color_inactive = pygame.Color('lightskyblue3')
        self.color_active = pygame.Color('dodgerblue2')
        
        # 0 = xyz input
        # 1 = single value input
        # 2 = xy input
        self.type = type
        
    def update_active(self, event_pos):
        # 0 = xyz input
        if self.type == 0:
            if self.x_input_rect.collidepoint(event_pos):
                self.x_active = True
                self.y_active = False
                self.z_active = False
            elif self.y_input_rect.collidepoint(event_pos):
                self.x_active = False
                self.y_active = True
                self.z_active = False
            elif self.z_input_rect.collidepoint(event_pos):
                self.x_active = False
                self.y_active = False
                self.z_active = True
            else:
                self.x_active = False
                self.y_active = False
                self.z_active = False
        # 1 = single value input
        elif self.type == 1:
            if self.x_input_rect.collidepoint(event_pos):
                self.x_active = True
            else:
                self.x_active = False
        # 2 = xy input
        elif self.type == 2:
            if self.x_input_rect.collidepoint(event_pos):
                self.x_active = True
                self.y_active = False
            elif self.y_input_rect.collidepoint(event_pos):
                self.x_active = False
                self.y_active = True
            else:
                self.x_active = False
                self.y_active = False

    def update_input(self, event):
        if self.x_active:
            if event.key == pygame.K_BACKSPACE:
                self.x_input = self.x_input[:-1]
            else:
                self.x_input += event.unicode
        elif self.y_active:
            if event.key == pygame.K_BACKSPACE:
                self.y_input = self.y_input[:-1]
            else:
                self.y_input += event.unicode
        elif self.z_active:
            if event.key == pygame.K_BACKSPACE:
                self.z_input = self.z_input[:-1]
            else:
                self.z_input += event.unicode

    def draw(self):
        """Draws the input box and the text that has been entered so far."""

        # draws label
        self.screen.blit(self.label, self.label_rect)
        
        # 0 = xyz input
        if self.type == 0:

            # designates colors of boxes
            x_color = self.color_active if self.x_active else self.color_inactive
            y_color = self.color_active if self.y_active else self.color_inactive
            z_color = self.color_active if self.z_active else self.color_inactive

            # Render the current text.
            x_txt_surface = self.font.render(self.x_input, True, x_color)
            y_txt_surface = self.font.render(self.y_input, True, y_color)
            z_txt_surface = self.font.render(self.z_input, True, z_color)
            
            # Resize the xyz boxes if the text is too long.
            x_width = max(75, x_txt_surface.get_width()+10)
            self.x_input_rect.w = x_width
            y_width = max(75, y_txt_surface.get_width()+10)
            self.y_input_rect.w = y_width
            z_width = max(75, z_txt_surface.get_width()+10)
            self.z_input_rect.w = z_width

            # Blit the Labels
            self.screen.blit(self.x_label, self.x_label_rect)
            self.screen.blit(self.y_label, self.y_label_rect)
            self.screen.blit(self.z_label, self.z_label_rect)

            # Blit the text.
            self.screen.blit(x_txt_surface, (self.x_input_rect.x+5, self.x_input_rect.y+5))
            self.screen.blit(y_txt_surface, (self.y_input_rect.x+5, self.y_input_rect.y+5))
            self.screen.blit(z_txt_surface, (self.z_input_rect.x+5, self.z_input_rect.y+5))
            
            # Blit the xyz input_box rect.
            pygame.draw.rect(self.screen, x_color, self.x_input_rect, 2)
            pygame.draw.rect(self.screen, y_color, self.y_input_rect, 2)
            pygame.draw.rect(self.screen, z_color, self.z_input_rect, 2)
        # 1 = single value input
        elif self.type == 1:

            # designates color of the box
            color = self.color_active if self.x_active else self.color_inactive
           
            # Render the current text.
            txt_surface = self.font.render(self.x_input, True, color)

            # Resize the xyz boxes if the text is too long.
            width = max(75, txt_surface.get_width()+10)
            self.x_input_rect.w = width

            # Blit the text.
            self.screen.blit(txt_surface, (self.x_input_rect.x+5, self.x_input_rect.y+5))

            # Blit the xyz input_box rect.
            pygame.draw.rect(self.screen, color, self.x_input_rect, 2)
        # 2 = xy input
        elif self.type == 2:
            # designates colors of boxes
            x_color = self.color_active if self.x_active else self.color_inactive
            y_color = self.color_active if self.y_active else self.color_inactive

            # Render the current text.
            x_txt_surface = self.font.render(self.x_input, True, x_color)
            y_txt_surface = self.font.render(self.y_input, True, y_color)
            
            # Resize the xyz boxes if the text is too long.
            x_width = max(75, x_txt_surface.get_width()+10)
            self.x_input_rect.w = x_width
            y_width = max(75, y_txt_surface.get_width()+10)
            self.y_input_rect.w = y_width

            # Blit the Labels
            self.screen.blit(self.x_label, self.x_label_rect)
            self.screen.blit(self.y_label, self.y_label_rect)

            # Blit the text.
            self.screen.blit(x_txt_surface, (self.x_input_rect.x+5, self.x_input_rect.y+5))
            self.screen.blit(y_txt_surface, (self.y_input_rect.x+5, self.y_input_rect.y+5))

            # Blit the xyz input_box rect.
            pygame.draw.rect(self.screen, x_color, self.x_input_rect, 2)
            pygame.draw.rect(self.screen, y_color, self.y_input_rect, 2)
        
if __name__ == "__main__":
    run_skydiver_simulation(True)