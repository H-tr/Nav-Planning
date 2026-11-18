#pragma once

#include <vamp/vector.hh>
#include <vamp/vector/math.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots
{
struct Tortobot
{
    static constexpr char* name = "tortobot";
    static constexpr std::size_t dimension = 4;
    static constexpr std::size_t n_spheres = 1;
    static constexpr float min_radius = 0.3199999928474426;
    static constexpr float max_radius = 0.3199999928474426;
    static constexpr std::size_t resolution = 64;

    static constexpr std::array<std::string_view, dimension> joint_names = {"base_x_joint", "base_y_joint", "base_theta_joint", "camera_pan"};
    static constexpr char* end_effector = "camera_link";

    using Configuration = FloatVector<dimension>;
    using ConfigurationArray = std::array<FloatT, dimension>;

    struct alignas(FloatVectorAlignment) ConfigurationBuffer
        : std::array<float, Configuration::num_scalars_rounded>
    {
    };

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, n_spheres> x;
        FloatVector<rake, n_spheres> y;
        FloatVector<rake, n_spheres> z;
        FloatVector<rake, n_spheres> r;
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_m{
        20.0, 20.0, 6.2831854820251465, 6.2831854820251465
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_a{
        -10.0, -10.0, -3.1415927410125732, -3.1415927410125732
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> d_m{
        0.05000000074505806, 0.05000000074505806, 0.15915493667125702, 0.15915493667125702
    };

    static inline void scale_configuration(Configuration& q) noexcept
    {
        q = q * Configuration(s_m) + Configuration(s_a);
    }

    static inline void descale_configuration(Configuration& q) noexcept
    {
        q = (q - Configuration(s_a)) * Configuration(d_m);
    }

    template <std::size_t rake>
    static inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = -10.0 + (q[0] * 20.0);
q[1] = -10.0 + (q[1] * 20.0);
q[2] = -3.1415927410125732 + (q[2] * 6.2831854820251465);
q[3] = -3.1415927410125732 + (q[3] * 6.2831854820251465);

    }

    template <std::size_t rake>
    static inline void descale_configuration_block(ConfigurationBlock<rake> & q) noexcept
    {
        q[0] = 0.05000000074505806 * (q[0] - -10.0);
q[1] = 0.05000000074505806 * (q[1] - -10.0);
q[2] = 0.15915493667125702 * (q[2] - -3.1415927410125732);
q[3] = 0.15915493667125702 * (q[3] - -3.1415927410125732);

    }

    inline static auto space_measure() noexcept -> float
    {
        return 15791.367920612629;
    }

    template <std::size_t rake>
    static inline void sphere_fk(const ConfigurationBlock<rake> &x, Spheres<rake> &out) noexcept
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 4> y;

           // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = 0.;
   y[3] = 0.319999992847443;


        for (auto i = 0U; i < 1; ++i)
        {
            out.x[i] = y[i * 4 + 0];
            out.y[i] = y[i * 4 + 1];
            out.z[i] = y[i * 4 + 2];
            out.r[i] = y[i * 4 + 3];
        }
    }

    using Debug = std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    template <std::size_t rake>
        static inline auto fkcc_debug(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept -> Debug
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 8> y;

           // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = 0.;
   y[3] = 0.319999992847443;
   y[4] = x[0];
   y[5] = x[1];
   y[6] = 0.;
   y[7] = 0.319999992847443;


        Debug output;

        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[0],
                y[1],
                y[2],
                y[3]));
        

        

        return output;
    }

    template <std::size_t rake>
        static inline bool fkcc(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 8> y;

           // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = 0.;
   y[3] = 0.319999992847443;
   y[4] = x[0];
   y[5] = x[1];
   y[6] = 0.;
   y[7] = 0.319999992847443;

        





//
// environment vs. robot collisions
//

// base_link
if (sphere_environment_in_collision(environment,
                                    y[4],
                                    y[5],
                                    y[6],
                                    y[7]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        return true;
    }

    template <std::size_t rake>
    static inline bool fkcc_attach(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 5> v;
        std::array<FloatVector<rake, 1>, 20> y;

           v[0] = cos(x[2]);
   v[1] = cos(x[3]);
   v[2] = sin(x[2]);
   v[3] = 0. - v[2];
   v[4] = sin(x[3]);
   y[11] = v[0] * v[1] + v[3] * v[4];
   y[12] = v[2] * v[1] + v[0] * v[4];
   v[4] = 0. - v[4];
   y[14] = v[0] * v[4] + v[3] * v[1];
   y[15] = v[2] * v[4] + v[0] * v[1];
   // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = 0.;
   y[3] = 0.319999992847443;
   y[4] = x[0];
   y[5] = x[1];
   y[6] = 0.;
   y[7] = 0.319999992847443;
   y[8] = x[0];
   y[9] = x[1];
   y[10] = 0.08;
   y[13] = 0.;
   y[16] = 0.;
   y[17] = 0.;
   y[18] = 0.;
   y[19] = 1.;

        





//
// environment vs. robot collisions
//

// base_link
if (sphere_environment_in_collision(environment,
                                    y[4],
                                    y[5],
                                    y[6],
                                    y[7]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        // attaching at camera_link
        set_attachment_pose(environment, to_isometry(&y[8]));

        //
        // attachment vs. environment collisions
        //
        if (attachment_environment_collision(environment))
        {
            return false;
        }

        //
        // attachment vs. robot collisions
        //

        

        return true;
    }

    static inline auto eefk(const std::array<float, 4> &x) noexcept -> Eigen::Isometry3f
    {
        std::array<float, 5> v;
        std::array<float, 12> y;

           v[0] = cos(x[2]);
   v[1] = cos(x[3]);
   v[2] = sin(x[2]);
   v[3] = 0. - v[2];
   v[4] = sin(x[3]);
   y[3] = v[0] * v[1] + v[3] * v[4];
   y[4] = v[2] * v[1] + v[0] * v[4];
   v[4] = 0. - v[4];
   y[6] = v[0] * v[4] + v[3] * v[1];
   y[7] = v[2] * v[4] + v[0] * v[1];
   // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = 0.08;
   y[5] = 0.;
   y[8] = 0.;
   y[9] = 0.;
   y[10] = 0.;
   y[11] = 1.;


        return to_isometry(y.data());
    }
};
}

// NOLINTEND(*-magic-numbers)
