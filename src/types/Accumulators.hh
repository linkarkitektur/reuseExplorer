#pragma once
#include <vector>
#include <functional>
#include <pcl/common/impl/accumulators.hpp>


namespace linkml {


    struct AccumulatorConfidence
    {

      // Requires that point type has label field
      using IsCompatible = pcl::traits::has_label<boost::mpl::_1>;

      // Storage
      // A better performance may be achieved with a heap structure
      std::map<std::uint32_t, std::size_t> labels;

      template <typename PointT> void
      add (const PointT& t)
      {
        auto itr = labels.find (t.confidence);
        if (itr == labels.end ())
          labels.insert (std::make_pair (t.confidence, 1));
        else
          ++itr->second;
      }

      template <typename PointT> void
      get (PointT& t, std::size_t) const
      {
        std::size_t max = 0;
        for (const auto &label : labels)
          if (label.second > max)
          {
            max = label.second;
            t.confidence = label.first;
          }
      }

    };
    struct AccumulatorSemantic
        {

        // Requires that point type has label field
        using IsCompatible = pcl::traits::has_label<boost::mpl::_1>;

        // Storage
        // A better performance may be achieved with a heap structure
        std::map<std::uint32_t, std::size_t> labels;

        template <typename PointT> void
        add (const PointT& t)
        {
            auto itr = labels.find (t.semantic);
            if (itr == labels.end ())
            labels.insert (std::make_pair (t.semantic, 1));
            else
            ++itr->second;
        }

        template <typename PointT> void
        get (PointT& t, std::size_t) const
        {
          std::size_t max = 0;
          bool filter_zero = false;

          if (labels.size() >= 2)
            if (labels.find(0) != labels.end())
                filter_zero = true;

          for (const auto &label : labels){
            if (filter_zero && label.first == 0)
                continue;

            if (label.second > max){
                max = label.second;
                t.semantic = label.first;
            }
          }
        }

        };
    struct AccumulatorInstance
    {

        // Requires that point type has label field
        using IsCompatible = pcl::traits::has_label<boost::mpl::_1>;

        // Storage
        // A better performance may be achieved with a heap structure
        std::map<std::uint32_t, std::size_t> labels;

        template <typename PointT> void
        add (const PointT& t)
        {
            auto itr = labels.find (t.instance);
            if (itr == labels.end ())
            labels.insert (std::make_pair (t.instance, 1));
            else
            ++itr->second;
        }

        template <typename PointT> void
        get (PointT& t, std::size_t) const
        {
            std::size_t max = 0;
            for (const auto &label : labels)
            if (label.second > max)
            {
                max = label.second;
                t.instance = label.first;
            }
        }

    };

    /* Meta-function that checks if an accumulator is compatible with given
     * point type(s). */
    template <typename Point1T, typename Point2T = Point1T>
    struct IsAccumulatorCompatible {

      template <typename AccumulatorT>
      struct apply : boost::mpl::and_<
                       boost::mpl::apply<typename AccumulatorT::IsCompatible, Point1T>,
                       boost::mpl::apply<typename AccumulatorT::IsCompatible, Point2T>
                     > {};
    };

    /* Meta-function that creates a Fusion vector of accumulator types that are
     * compatible with a given point type. */
    template <typename PointT>
    struct Accumulators
    {
      using type =
        typename boost::fusion::result_of::as_vector<
          typename boost::mpl::filter_view<
            boost::mpl::vector<
              pcl::detail::AccumulatorXYZ
            , pcl::detail::AccumulatorNormal
            , pcl::detail::AccumulatorCurvature
            , pcl::detail::AccumulatorRGBA
            , pcl::detail::AccumulatorIntensity
            , pcl::detail::AccumulatorLabel
            , AccumulatorConfidence
            , AccumulatorSemantic
            , AccumulatorInstance
            >
          , IsAccumulatorCompatible<PointT>
          >
        >::type;
    };

    // /* Fusion function object to invoke point addition on every accumulator in
    //  * a fusion sequence. */
    // template <typename PointT>
    // struct AddPoint
    // {

    //   const PointT& p;

    //   AddPoint (const PointT& point) : p (point) { }

    //   template <typename AccumulatorT> void
    //   operator () (AccumulatorT& accumulator) const
    //   {
    //     accumulator.add (p);
    //   }

    // };

    // /* Fusion function object to invoke get point on every accumulator in a
    //  * fusion sequence. */
    // template <typename PointT>
    // struct GetPoint
    // {

    //   PointT& p;
    //   std::size_t n;

    //   GetPoint (PointT& point, std::size_t num) : p (point), n (num) { }

    //   template <typename AccumulatorT> void
    //   operator () (AccumulatorT& accumulator) const
    //   {
    //     accumulator.get (p, n);
    //   }

    // };
} // namespace linkml