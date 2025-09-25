// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <open3d/Open3D.h>

#ifndef FLEXIV_RECONSTRUCTION_PROPERTY_PANEL_HPP_
#define FLEXIV_RECONSTRUCTION_PROPERTY_PANEL_HPP_

using namespace open3d;
using namespace open3d::visualization;

namespace flexiv {
namespace reconstruction {

/**
 * @class PropertyPanel
 * @brief Side panel in the reconstruction GUI to online tune the parameters.
 */
class PropertyPanel : public gui::VGrid
{
public:
    PropertyPanel(int spacing, int left_margin);
    virtual ~PropertyPanel() = default;

    void AddBool(const std::string& name, std::atomic<bool>* bool_addr,
        bool default_val, const std::string& tooltip = "");

    void AddFloatSlider(const std::string& name, std::atomic<double>* num_addr,
        double default_val, double min_val, double max_val,
        const std::string& tooltip = "");

    void AddIntSlider(const std::string& name, std::atomic<int>* num_addr,
        int default_val, int min_val, int max_val,
        const std::string& tooltip = "");

    void AddValues(const std::string& name, std::atomic<int>* idx_addr,
        int default_idx, std::vector<std::string> values,
        const std::string& tooltip = "");

    void SetEnabled(bool enable) override;

    void SetOnChanged(std::function<void()> f) { on_changed_ = f; }

private:
    gui::Color default_label_color_;
    std::function<void()> on_changed_;

    void NotifyChanged()
    {
        if (on_changed_) {
            on_changed_();
        }
    }
};

} /* namespace reconstruction */
} /* namespace flexiv */

#endif /* FLEXIV_RECONSTRUCTION_PROPERTY_PANEL_HPP_ */