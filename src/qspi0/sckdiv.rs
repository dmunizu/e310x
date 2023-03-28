#[doc = "Register `sckdiv` reader"]
pub struct R(crate::R<SCKDIV_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<SCKDIV_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<SCKDIV_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<SCKDIV_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Register `sckdiv` writer"]
pub struct W(crate::W<SCKDIV_SPEC>);
impl core::ops::Deref for W {
    type Target = crate::W<SCKDIV_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl core::ops::DerefMut for W {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl From<crate::W<SCKDIV_SPEC>> for W {
    #[inline(always)]
    fn from(writer: crate::W<SCKDIV_SPEC>) -> Self {
        W(writer)
    }
}
#[doc = "Field `div` reader - Divisor for serial clock"]
pub type DIV_R = crate::FieldReader<u16, u16>;
#[doc = "Field `div` writer - Divisor for serial clock"]
pub type DIV_W<'a, const O: u8> = crate::FieldWriter<'a, u32, SCKDIV_SPEC, u16, u16, 12, O>;
impl R {
    #[doc = "Bits 0:11 - Divisor for serial clock"]
    #[inline(always)]
    pub fn div(&self) -> DIV_R {
        DIV_R::new((self.bits & 0x0fff) as u16)
    }
}
impl W {
    #[doc = "Bits 0:11 - Divisor for serial clock"]
    #[inline(always)]
    pub fn div(&mut self) -> DIV_W<0> {
        DIV_W::new(self)
    }
    #[doc = "Writes raw bits to the register."]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.0.bits(bits);
        self
    }
}
#[doc = "Serial Clock Divisor Register\n\nThis register you can [`read`](crate::generic::Reg::read), [`write_with_zero`](crate::generic::Reg::write_with_zero), [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`modify`](crate::generic::Reg::modify). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [sckdiv](index.html) module"]
pub struct SCKDIV_SPEC;
impl crate::RegisterSpec for SCKDIV_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [sckdiv::R](R) reader structure"]
impl crate::Readable for SCKDIV_SPEC {
    type Reader = R;
}
#[doc = "`write(|w| ..)` method takes [sckdiv::W](W) writer structure"]
impl crate::Writable for SCKDIV_SPEC {
    type Writer = W;
}
#[doc = "`reset()` method sets sckdiv to value 0"]
impl crate::Resettable for SCKDIV_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0
    }
}
