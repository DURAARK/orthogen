/*
 * Simple Meanshift implementation
 */

#include <set>
#include <vector>

typedef Eigen::Vector3d Vec3d;

class Meanshift
{
private:
	std::vector < Vec3d > points;
	std::vector < Vec3d > window;

public:
	template< typename VTYPE >
	struct EigenLexicographicalComparator
	{
		inline bool operator() (const VTYPE &lhs, const VTYPE &rhs) const
		{
			for (int i = 0; i < VTYPE::RowsAtCompileTime; ++i)
			{
				if (lhs[i] < rhs[i]) return true;
				if (lhs[i] > rhs[i]) return false;
			}
			return false;
		}
	};


	// perform meanshift, rectangular window, one window per point
	// very simple and slow
	void calculate(const Vec3d &pmin, const Vec3d &pmax, const double windowwidth)
	{
		auto const inslab = [windowwidth](const double val, const double center) -> bool
		{
			return (val < (center - windowwidth)) || (val >(center + windowwidth));
		};

		std::vector<bool> converged;
		converged.resize(points.size(), false);

		window = points;

		bool allconverged = false;
		
		while (!allconverged)
		{
			allconverged = true;
			int i = 0;
			for (auto &c : window)
			{
				if (!converged[i])
				{
					std::vector<Vec3d> inpoints;
					// filter points inside window
					for (auto const &p : points)
					{
						if (inslab(p[0], c[0]) && inslab(p[1], c[1]) && inslab(p[2], c[2]))
						{
							inpoints.push_back(p);
						}
					}
					// calculate mean shift
					Vec3d mean = Vec3d(0.0, 0.0, 0.0);
					for (auto const &p : points) mean += p;
					mean /= (double)points.size();

					Vec3d ms = mean - c;
					if (sqrt(ms[0] * ms[0] + ms[1] * ms[1] + ms[2] * ms[2]) < 0.01)
					{
						converged[i] = true;
					}
					else 
					{
						allconverged = false;
						c += mean - c;
					}
				}
				++i; // window index
			}
		}
	}

};
