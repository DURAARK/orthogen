/*
 * Simple Meanshift implementation
 */

#include <set>
#include <vector>

typedef Eigen::Vector3d Vec3d;

class Meanshift
{

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

	std::vector < std::pair < Vec3d, int > > points;
	std::set < Vec3d, EigenLexicographicalComparator< Vec3d > > clustercenters;

	// give minimum and maximum, as well as number of clusters per dimension
	// (will perform regular sampling)
	void calculate(const Vec3d &pmin, const Vec3d &pmax, const double clusterwidth)
	{
		const Vec3d clusters = (pmax - pmin) / clusterwidth;
		const int clustersx = (int)ceil(clusters[0]);
		const int clustersy = (int)ceil(clusters[1]);
		const int clustersz = (int)ceil(clusters[2]);
		const double cwx = clusters[0] - floor(clusters[0]);
		const double cwy = clusters[1] - floor(clusters[1]);
		const double cwz = clusters[2] - floor(clusters[2]);
		for (int iz = 0; iz < clustersz; ++iz)
		{
			for (int iy = 0; iy < clustersy; ++iy)
			{
				for (int ix = 0; ix < clustersx; ++ix)
				{
					// create a cluster
					clustercenters.insert(
						Vec3d( 
							pmin[0] + (cwx/2.0) + ix*clusterwidth,
							pmin[1] + (cwy/2.0) + iy*clusterwidth,
							pmin[2] + (cwz/2.0) + iz*clusterwidth
						) );

				}
			}
		}
	}

};
