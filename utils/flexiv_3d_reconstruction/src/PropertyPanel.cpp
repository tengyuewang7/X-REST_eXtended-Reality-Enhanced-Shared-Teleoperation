#include <flexiv/reconstruction/PropertyPanel.hpp>

namespace flexiv {
namespace reconstruction {

PropertyPanel::PropertyPanel(int spacing, int left_margin)
: gui::VGrid(2, spacing, gui::Margins(left_margin, 0, 0, 0))
{
    default_label_color_ = std::make_shared<gui::Label>("temp")->GetTextColor();
}

void PropertyPanel::AddBool(const std::string& name,
    std::atomic<bool>* bool_addr, bool default_val, const std::string& tooltip)
{
    auto cb = std::make_shared<gui::Checkbox>("");
    cb->SetChecked(default_val);
    *bool_addr = default_val;
    cb->SetOnChecked([bool_addr, this](bool is_checked) {
        *bool_addr = is_checked;
        this->NotifyChanged();
    });
    auto label = std::make_shared<gui::Label>(name.c_str());
    label->SetTooltip(tooltip.c_str());
    AddChild(label);
    AddChild(cb);
}

void PropertyPanel::AddFloatSlider(const std::string& name,
    std::atomic<double>* num_addr, double default_val, double min_val,
    double max_val, const std::string& tooltip)
{
    auto s = std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
    s->SetLimits(min_val, max_val);
    s->SetValue(default_val);
    *num_addr = default_val;
    s->SetOnValueChanged([num_addr, this](double new_val) {
        *num_addr = new_val;
        this->NotifyChanged();
    });
    auto label = std::make_shared<gui::Label>(name.c_str());
    label->SetTooltip(tooltip.c_str());
    AddChild(label);
    AddChild(s);
}

void PropertyPanel::AddIntSlider(const std::string& name,
    std::atomic<int>* num_addr, int default_val, int min_val, int max_val,
    const std::string& tooltip)
{
    auto s = std::make_shared<gui::Slider>(gui::Slider::INT);
    s->SetLimits(min_val, max_val);
    s->SetValue(default_val);
    *num_addr = default_val;
    s->SetOnValueChanged([num_addr, this](int new_val) {
        *num_addr = new_val;
        this->NotifyChanged();
    });
    auto label = std::make_shared<gui::Label>(name.c_str());
    label->SetTooltip(tooltip.c_str());
    AddChild(label);
    AddChild(s);
}

void PropertyPanel::AddValues(const std::string& name,
    std::atomic<int>* idx_addr, int default_idx,
    std::vector<std::string> values, const std::string& tooltip)
{
    auto combo = std::make_shared<gui::Combobox>();
    for (auto& v : values) {
        combo->AddItem(v.c_str());
    }
    combo->SetSelectedIndex(default_idx);
    *idx_addr = default_idx;
    combo->SetOnValueChanged(
        [idx_addr, this](const char* new_value, int new_idx) {
            *idx_addr = new_idx;
            this->NotifyChanged();
        });
    auto label = std::make_shared<gui::Label>(name.c_str());
    label->SetTooltip(tooltip.c_str());
    AddChild(label);
    AddChild(combo);
}

void PropertyPanel::SetEnabled(bool enable)
{
    gui::VGrid::SetEnabled(enable);
    for (auto child : GetChildren()) {
        child->SetEnabled(enable);
        auto label = std::dynamic_pointer_cast<gui::Label>(child);
        if (label) {
            if (enable) {
                label->SetTextColor(default_label_color_);
            } else {
                label->SetTextColor(gui::Color(0.5f, 0.5f, 0.5f, 1.0f));
            }
        }
    }
}

} /* namespace reconstruction */
} /* namespace flexiv */