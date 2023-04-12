// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>

#include "assert.h"
#include "world.hpp"

#include "math/tiny/tiny_dual.h"
#include "math/tiny/tiny_matrix3x3.h"
#include "math/tiny/tiny_quaternion.h"
#include "math/tiny/tiny_vector3.h"
#include "math/tiny/fix64_scalar.h"

#include "dynamics/kinematics.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "utils/pendulum.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "multi_body.hpp"
#include "world.hpp"

using namespace TINY;
using namespace tds;
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "math/tiny/tiny_algebra.hpp"

#include <chrono>  // std::chrono::seconds
#include <thread>  // std::this_thread::sleep_for



#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_dual_double_utils.h"
#include "math/pose.hpp"
#include "multi_body.hpp"
#include "rigid_body.hpp"

#include "utils/file_utils.hpp"
#include <random>

std::string sphere2red;


// ID of the ball whose position is optimized for
//const int TARGET_ID = 5;
// Constants
const double target_amplitude = 2;
const double target_frequency = 0.1;

template <typename Algebra>
// basically, error for ball position after 300 steps when we apply force x and y + do a render of it
// CREATE ENVIRONMENT + apply force + nb of steps (frames) and calculate the sqrn error after all these frames are done
typename Algebra::Scalar rollout(std::vector<double> t_force_x,  std::vector<double> t_force_y, int steps = 300, TinyOpenGL3App* app=0,typename Algebra::Scalar dt = Algebra::fraction(1, 60)) {

  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  typedef RigidBody<Algebra> TinyRigidBody;
  typedef Geometry<Algebra> TinyGeometry;

  std::vector<int> visuals;
 

  int sphere_shape = -1;
  if (app) {
      app->m_instancingRenderer->remove_all_instances();
      sphere_shape = app->register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
  }

  Scalar gravity_z = Algebra::zero();
  World<Algebra> world(gravity_z);

  std::vector<TinyRigidBody*> bodies;

  Scalar radius = Algebra::half();
  Scalar mass = Algebra::one();
  Scalar deg_60 = Algebra::pi() / Algebra::fraction(3, 1);  // even triangle
  Scalar dx = Algebra::cos(deg_60) * radius * Algebra::two();
  Scalar dy = Algebra::sin(deg_60) * radius * Algebra::two();
  Scalar rx = Algebra::zero(), y = Algebra::zero();

  //TARGET POSITION [3.5, 8, 0]
  // Vector3 target(Algebra::fraction(35, 10),Algebra::fraction(8, 1), Algebra::zero());
  Vector3 target = Vector3::create(Algebra::zero(), Algebra::zero(), Algebra::zero());
  Vector3 white_position = Vector3::create(Algebra::zero(), Algebra::zero(), Algebra::zero());

  const TinyGeometry* white_geom = world.create_sphere(radius);
  TinyRigidBody* white_ball = world.create_rigid_body(mass, white_geom);
  white_ball->world_pose_.position_ = white_position;

  bodies.push_back(white_ball);

  // render multiple frames for every step
  Scalar cost = 0;
  // RENDER done once
  for (int i = 0; i < steps; i++) {
    // apply force 
    // does this update the bodies[white_ball] ??
    bodies[0]->apply_central_force(Vector3::create(t_force_x[i], t_force_y[i], Algebra::zero()));
    // white_ball->apply_central_force(Vector3::create(t_force_x[i], t_force_y[i], Algebra::zero()));
    target = Vector3::create( target_amplitude * sin(target_frequency * i), target_amplitude * sin(target_frequency * i), target.z());

    if (app) {
        //reset
        for (int j = 0; j < visuals.size(); j++) {
          app->m_renderer->remove_graphics_instance(j);
        }
        visuals.clear();
        {
            // TinyVector3f pos(Algebra::to_double(white.x()), Algebra::to_double(white.y()),Algebra::to_double(white.z()));
            //TODO set these attributes only once
            TinyVector3f pos(
              Algebra::to_double(bodies[0]->world_pose_.position_.getX()),
              Algebra::to_double(bodies[0]->world_pose_.position_.getY()),
              Algebra::to_double(bodies[0]->world_pose_.position_.getZ()));
            TinyQuaternionf orn(0, 0, 0, 1);
            TinyVector3f color(1, 1, 1); //white ball
            TinyVector3f scaling(0.5, 0.5, 0.5);
            int instance = app->m_renderer->register_graphics_instance(sphere_shape, pos, orn, color, scaling);
            visuals.push_back(instance);
        }
        {
            //move the target's position with sin (blue ball)
            //TODO set variable only once and add it in bodies (only modify the position)
            //move target on y axis and x axis
            TinyVector3f pos(Algebra::to_double(target.x()), Algebra::to_double(target.y()),Algebra::to_double(target.z()));
            TinyQuaternionf orn(0, 0, 0, 1);
            TinyVector3f color(0, 0, 1);// blue ball current target
            TinyVector3f scaling(0.5, 0.5, 0.5);
            int instance = app->m_renderer->register_graphics_instance(sphere_shape, pos, orn, color, scaling);
            visuals.push_back(instance);
        }
    }
    
    world.step(dt);
    int upAxis = 2;
    if (app) {
      app->m_renderer->update_camera(upAxis);
      DrawGridData data;
      data.drawAxis = true;
      data.upAxis = upAxis;
      app->draw_grid(data);
      double dtd = Algebra::to_double(dt);
      // update visualization
      std::this_thread::sleep_for(std::chrono::duration<double>(dtd));
      //send bodies to CPU
      for (int b = 0; b < bodies.size(); b++) {
        const TinyRigidBody* body = bodies[b];
        int sphere_id = visuals[b];
          TinyVector3f base_pos(
              Algebra::to_double(body->world_pose_.position_.getX()),
              Algebra::to_double(body->world_pose_.position_.getY()),
              Algebra::to_double(body->world_pose_.position_.getZ()));
          TinyQuaternionf base_orn(
              Algebra::to_double(body->world_pose_.orientation_.getX()),
              Algebra::to_double(body->world_pose_.orientation_.getY()),
              Algebra::to_double(body->world_pose_.orientation_.getZ()),
              Algebra::to_double(body->world_pose_.orientation_.getW()));
          app->m_instancingRenderer->write_single_instance_transform_to_cpu(base_pos, base_orn, sphere_id);
      }
      // render the whole scene
      app->m_renderer->render_scene();
      app->m_renderer->write_transforms();
      app->swap_buffer();
    }
    // Calculate error based on force applied in t_force, only once ?
    // cost += (bodies[0]->world_pose_.position_ - target).length_squared();
    cost += (bodies[0]->world_pose_.position_ - target).sqnorm();
   /* double x = t_force_x[i] - target_[i]
    cost += std::sqrt(dx * dx + dy * dy);*/
  }
  return cost;
}

/// Computes gradient using finite differences
void grad_finite(std::vector<double>& t_force_x, std::vector<double>& t_force_y, double* cost, std::vector<double>* grad_force_x, std::vector<double>* grad_force_y, int steps = 300, double eps=1e-5 ) {
   //count the normal cost
  *cost = rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y, steps);
  std::vector<double> t_force_x_original = t_force_x;
  std::vector<double> t_force_y_original = t_force_y;
  
 // Compute gradient with respect to t_force_x
  for (int i = 0; i < steps; i++) {
    // Perturb t_force_x
    t_force_x[i] += eps;

    // Compute cost with perturbed t_force_x
    double cx = rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y, steps);

    // Restore t_force_x to original value
    t_force_x[i] = t_force_x_original[i];

    // Perturb t_force_y
    t_force_y[i] += eps;

    // Compute cost with perturbed t_force_y
    double cy =  rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y, steps);

    // Restore t_force_y to original value
    t_force_y[i] = t_force_y_original[i];

    // Compute gradients
    (*grad_force_x)[i] = (cx - *cost) / eps;
    (*grad_force_y)[i] = (cy - *cost) / eps;
  }

  // Update cost with original forces
  *cost =  rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y, steps);
}

void adam_optimizer(std::vector<double>& t_force_x,
                    std::vector<double>& t_force_y, double* cost,
                    std::vector<double>* grad_force_x,
                    std::vector<double>* grad_force_y, int steps = 300,
                    double lr = 0.01, double beta1 = 0.9, double beta2 = 0.999,
                    double eps = 1e-8,
                    double minCost = 50,
                    int max_iterations = 10000, 
                    TinyOpenGL3App* app = 0) {
  std::vector<double> m_force_x(steps, 0.0);
  std::vector<double> m_force_y(steps, 0.0);
  std::vector<double> v_force_x(steps, 0.0);
  std::vector<double> v_force_y(steps, 0.0);
  int t = 0;
  while (*cost > minCost && t < max_iterations) {
    grad_finite(t_force_x, t_force_y, cost, grad_force_x, grad_force_y, steps);

    for (int i = 0; i < steps; i++) {

      m_force_x[i] = beta1 * m_force_x[i] + (1 - beta1) * (*grad_force_x)[i];
      m_force_y[i] = beta1 * m_force_y[i] + (1 - beta1) * (*grad_force_y)[i];
      v_force_x[i] = beta2 * v_force_x[i] +
                     (1 - beta2) * (*grad_force_x)[i] * (*grad_force_x)[i];
      v_force_y[i] = beta2 * v_force_y[i] +
                     (1 - beta2) * (*grad_force_y)[i] * (*grad_force_y)[i];
      double m_hat_force_x = m_force_x[i] / (1 - pow(beta1, t + 1));
      double m_hat_force_y = m_force_y[i] / (1 - pow(beta1, t + 1));
      double v_hat_force_x = v_force_x[i] / (1 - pow(beta2, t + 1));
      double v_hat_force_y = v_force_y[i] / (1 - pow(beta2, t + 1));
      (*grad_force_x)[i] = m_hat_force_x / (sqrt(v_hat_force_x) + eps);
      (*grad_force_y)[i] = m_hat_force_y / (sqrt(v_hat_force_y) + eps);
      t_force_x[i] -= lr * (*grad_force_x)[i];
      t_force_y[i] -= lr * (*grad_force_y)[i];
    }
    printf("Iteration %02d - cost: %.3f \n", t, *cost);
    if (app && t % 1000 == 0) {
        //get a preview every 1000 iterations
        rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y, steps,app);
    }
    t++;   
  }
}

// meilleur dans notre cas car, nous avons un probleme non-convexe aka pas 1 seule bonne solution
//void grad_dual(std::vector<double>& t_force_x, std::vector<double>& t_force_y, double* cost, std::vector<double>* grad_force_x, std::vector<double>* grad_force_y, int steps = 300, double eps=1e-5 ) {
//  
//    typedef TinyDual<double> TinyDual;
//    std::vector<TinyDual> t_force_x_dual(300);
//    std::vector<TinyDual> t_force_y_dual(300);
//    // PAS OPTIMISER ....
//    for (int i = 0; i < steps; i++) {
//        TinyDual fx(t_force_x[i], 0.);
//        TinyDual fy(t_force_y[i], 0.);
//        t_force_x_dual[i] = fx;
//        t_force_y_dual[i] = fy;
//    }
//
//    for (int i = 0; i < steps; i++) {
//      {
//        TinyDual fx(t_force_x[i], 1.);
//        t_force_x_dual[i] = fx;
//        // calculate rollout cost
//        TinyDual c = rollout<TinyAlgebra<TinyDual, TinyDualDoubleUtils>>(t_force_x_dual, t_force_y_dual, steps);
//        *cost = c.real();
//        (*grad_force_x)[i] = c.dual();
//      }
//      {
//        TinyDual fx(t_force_x[i], 1.);
//        TinyDual fy(t_force_y[i], 0.);
//
//        TinyDual c =  rollout<TinyAlgebra<TinyDual, TinyDualDoubleUtils>>(fx, fy, steps);
//        (*grad_force_y)[i] = c.dual();
//      }
//    }
//}

//using projected gradient descent
// we want the solution to satisfy certain constraints
// basically we project the gradient of the function onto the plausible region (made by constraint) every iterations
// in other words, we tae a step towards negative gradient and project onto possible solution
void grad_dual(std::vector<double>& t_force_x, std::vector<double>& t_force_y,
               double* cost, std::vector<double>* grad_force_x,
               std::vector<double>* grad_force_y, int steps = 300,
               double lr = 0.1, double eps = 1e-8, double minCost = 50,
               int max_iterations = 10000, double lambda = 0.01, TinyOpenGL3App* app = 0) {
  
  int t = 0;
  std::vector<double> grad_x(steps);
  std::vector<double> grad_y(steps);
  while (*cost > minCost && t < max_iterations) {
    grad_finite(t_force_x, t_force_y, cost, &grad_x, &grad_y, steps);

    for (int i = 0; i < steps; i++) {
       /* t_force_x[i] = std::max(0.0, t_force_x[i] - lambda * (*grad_force_x)[i]);
        t_force_y[i] = std::max(0.0, t_force_y[i] - lambda * (*grad_force_y)[i]);*/
        t_force_x[i] -= lr * grad_x[i];
        t_force_y[i] -= lr * grad_y[i];
    }
    for (int i = 0; i < steps; i++) {
       /* t_force_x[i] -= lambda * (*grad_force_x)[i];
        t_force_y[i] -= lambda * (*grad_force_y)[i];
        (*grad_force_x)[i] = (*grad_force_x)[i] + grad_x[i];
        (*grad_force_y)[i] = (*grad_force_y)[i] + grad_y[i];*/
        // accumulate gradients for next iteration
        grad_x[i] += lambda * grad_x[i];
        grad_y[i] += lambda * grad_y[i];
    }
    printf("Iteration %02d - cost: %.3f \n", t, *cost);
    if (app && t % 1000 == 0) {
        // get a preview every 1000 iterations
        rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y, steps, app);
    }
    t++;
  }
}

int main(int argc, char* argv[]) {

  // using Vector3 = typename Algebra::Vector3;

    // Load URDF
  FileUtils::find_file("sphere2red.urdf", sphere2red);
  using namespace std::chrono;
  // Load OPENGL
  TinyOpenGL3App app("billiard_opt_example_gui", 1024, 768);
  app.m_renderer->init();
  app.set_up_axis(2);
  // setup scene
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);

  // init forces
  // double init_force_x = 0., init_force_y = 500.;
  int steps = 300;
  double lr = 0.01;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-5.0, 5.0); // range: [0.0, 10.0)
  std::vector<double> t_force_x(300);
  std::vector<double> t_force_y(300);
  // init randomly the forces
  std::generate(t_force_x.begin(), t_force_x.end(), [&]() { return dis(gen); });
  std::generate(t_force_y.begin(), t_force_y.end(), [&]() { return dis(gen); });
  // init animation
  //rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y, steps, &app);
  
  // ************* Grad finite
  
  //{
  //  auto start = high_resolution_clock::now();
  //   //create variables to update
  //  std::vector<double> t_force_x_original = t_force_x; // TODO COPY ???
  //  std::vector<double> t_force_y_original = t_force_y;
  //  double cost = 9999999;
  //   // init gradients vectors
  //  std::vector<double> t_force_grad_x(300);
  //  std::vector<double> t_force_grad_y(300);
  //  int iter = 0;
  //  double limite = 50;
  //  while (cost > limite && iter < 2000) {
  //    grad_finite(t_force_x, t_force_y, &cost, &t_force_grad_x, &t_force_grad_y, steps);
  //    printf("Iteration %02d - cost: %.3f \n", iter, cost);
  //    iter++;
  //    for (int i = 0; i < steps; ++i) {
  //      t_force_x[i] -= lr * t_force_grad_x[i];
  //      t_force_y[i] -= lr * t_force_grad_y[i];
  //    }
  //  }
  //  auto stop = high_resolution_clock::now();
  //  auto duration = duration_cast<microseconds>(stop - start);
  //  printf("Finite differences took %ld microseconds.", static_cast<long>(duration.count()));
  //  // do final render with optimized forces
  //  cost = rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y, steps, &app);
  //}

  // ******* Adam
  //{
  //  auto start = high_resolution_clock::now();
  //  // create variables to update
  //  double cost = 9999999;
  //  // init gradients vectors
  //  std::vector<double> t_force_grad_x(300);
  //  std::vector<double> t_force_grad_y(300);

  //  adam_optimizer(t_force_x, t_force_y, &cost, &t_force_grad_x, &t_force_grad_y, steps, 0.1, 0.9, 0.999, 1e-8, 0.01, 10000, &app);

  //  auto stop = high_resolution_clock::now();
  //  auto duration = duration_cast<microseconds>(stop - start);
  //  printf("Adam took %ld microseconds.", static_cast<long>(duration.count()));
  //  // do final render with optimized forces
  //  cost = rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y,steps, &app);
  //}
  
  // ******** Grad Dual
  //{
  //  auto start = high_resolution_clock::now();
  //  // create variables to update
  //  double cost = 9999999;
  //  // init gradients vectors
  //  std::vector<double> t_force_grad_x(300);
  //  std::vector<double> t_force_grad_y(300);

  //  grad_dual(t_force_x, t_force_y, &cost, &t_force_grad_x, &t_force_grad_y, steps, 0.01, 1e-8, 0.01, 10000, 0.01, &app);

  //  auto stop = high_resolution_clock::now();
  //  auto duration = duration_cast<microseconds>(stop - start);
  //  printf("grad dual took %ld microseconds.", static_cast<long>(duration.count()));
  //  // do final render with optimized forces
  //  cost = rollout<TinyAlgebra<double, DoubleUtils>>(t_force_x, t_force_y,
  //                                                   steps, &app);
  //}
  return 0;
}
